#!/usr/bin/env python3

import time
import lgpio
import threading
import queue
import signal
import sys
import os
import pty
import subprocess
import serial
from dataclasses import dataclass
from typing import Optional, List

# -- 1. CONFIGURATION --

# Ultrasonic Sensor Configuration
@dataclass
class UltrasonicSensor:
    name: str
    pin: int
    last_reading: Optional[float] = None
    last_update: float = 0.0

SENSORS = [
    UltrasonicSensor("izquierda", pin=13),
    UltrasonicSensor("frente",    pin=19),
    UltrasonicSensor("derecha",   pin=26)
]

# Performance Tuning
MIN_MEASURE_INTERVAL = 0.03  # 33Hz per sensor
SENSOR_TIMEOUT = 0.05        # 50ms echo timeout
SPEED_OF_SOUND = 34300 / 1_000_000  # cm/us

# Spike Prime Configuration
SPIKE_PORT = '/dev/ttyACM0'
SPIKE_BAUD = 115200

# Global Control Flags
running = True
sensor_queue = queue.Queue()

# -- 2. SPIKE PRIME COMMUNICATION --

class SpikeController:
    """Handles serial communication and control of the Spike Prime Hub."""
    def __init__(self, port: str, baudrate: int):
        self.port = port
        self.baudrate = baudrate
        self.spike = None

    def connect(self) -> bool:
        """Connect to the Spike Hub, killing screen session if needed."""
        self._kill_screen_session()
        try:
            self.spike = serial.Serial(self.port, self.baudrate, timeout=1)
            print("Connected to Spike Prime.")
            self._initialize_robot_functions()
            return True
        except serial.SerialException as e:
            print(f"Error connecting to Spike: {e}")
            return False

    def disconnect(self):
        """Disconnects from the Spike Hub and releases the serial port."""
        if self.spike and self.spike.is_open:
            try:
                # Send Ctrl+C to interrupt any running script on Spike
                self.spike.write(b'\x03')
                self.send_command("motor.stop(port.B, stop=motor.COAST)")
                self.send_command("motor.stop(port.F, stop=motor.COAST)")
            except Exception as e:
                print(f"Error during Spike cleanup: {e}")
            finally:
                self.spike.close()
                print("Spike connection closed.")

    def send_command(self, cmd: str):
        """Sends a command to the Spike and clears the buffer."""
        if not self.spike:
            return
        self.spike.write(f"{cmd}\r".encode())
        self.spike.read_until() # Wait for echo

    def _kill_screen_session(self):
        """Forcefully kills a screen session on the serial port to free it."""
        if not os.path.exists(self.port):
            return
        try:
            master_fd, slave_fd = pty.openpty()
            proc = subprocess.Popen(
                ["screen", self.port, str(self.baudrate)],
                stdin=slave_fd, stdout=slave_fd, stderr=slave_fd, close_fds=True
            )
            time.sleep(1.0)
            # Sequence to kill the screen session
            os.write(master_fd, b'\x03')  # Ctrl+C
            time.sleep(0.1)
            os.write(master_fd, b'\x01')  # Ctrl+A
            time.sleep(0.1)
            os.write(master_fd, b'k')
            time.sleep(0.1)
            os.write(master_fd, b'y')
            time.sleep(0.1)
            proc.terminate()
            os.close(master_fd)
            os.close(slave_fd)
            print("Cleared existing screen session.")
        except Exception as e:
            print(f"Could not clear screen session: {e}")

    def _initialize_robot_functions(self):
        """Uploads the required Python functions to the Spike Hub."""
        print("Initializing Spike functions...")
        # Send library imports
        self.send_command("import motor, time")
        self.send_command("from hub import port, motion_sensor")
        # PD Controller function
        self.send_command("def pd(s1,s2,vel,kp,kd,ea):\n  error=s1-s2\n  et=(kp*error)+(kd*(error-ea))\n  motor.run_to_absolute_position(port.F,int(et),300,direction=motor.SHORTEST_PATH)\n  motor.set_duty_cycle(port.B,(-100)*vel)\n  return error")
        # Advance Straight function
        self.send_command("def ad(vel,ref,kp,kd,ea):\n  return pd(ref*10, motion_sensor.tilt_angles()[0], vel, kp, kd, ea)")
        self.send_command("ea=0") # Initialize previous error
        print("Spike is ready.")

    def advance(self, vel: float, ref: int, kp: float, kd: float):
        """Commands the robot to advance straight using the PD controller."""
        self.send_command(f"ea = ad({vel}, {ref}, {kp}, {kd}, ea)")

    def stop(self):
        """Stops the robot's motors."""
        self.send_command("motor.stop(port.B)")

# -- 3. RASPBERRY PI ULTRASONIC SENSORS --

def get_distance(h: int, pin: int) -> Optional[float]:
    """Measures distance using an HC-SR04 sensor with minimal latency."""
    try:
        lgpio.gpio_claim_output(h, pin)
        lgpio.gpio_write(h, pin, 0)
        time.sleep(0.000002)  # 2us
        lgpio.gpio_write(h, pin, 1)
        time.sleep(0.000010) # 10us
        lgpio.gpio_write(h, pin, 0)

        lgpio.gpio_claim_input(h, pin)

        timeout = time.time() + SENSOR_TIMEOUT
        while lgpio.gpio_read(h, pin) == 0:
            if time.time() > timeout:
                return None
        pulse_start = time.time()

        timeout = time.time() + SENSOR_TIMEOUT
        while lgpio.gpio_read(h, pin) == 1:
            if time.time() > timeout:
                return None
        pulse_end = time.time()

        distance = (pulse_end - pulse_start) * SPEED_OF_SOUND * 500_000
        return distance if 2.0 < distance < 400.0 else None
    except Exception:
        return None

def sensor_worker(h: int, sensor: UltrasonicSensor):
    """Worker thread to continuously read one ultrasonic sensor."""
    while running:
        current_time = time.time()
        if current_time - sensor.last_update >= MIN_MEASURE_INTERVAL:
            distance = get_distance(h, sensor.pin)
            if distance is not None:
                sensor.last_reading = distance
                sensor.last_update = current_time
                sensor_queue.put(sensor.name)
        time.sleep(0.001) # Prevent 100% CPU usage

# -- 4. SETUP & MAIN LOGIC --

def setup_gpio() -> Optional[int]:
    """Initializes lgpio and claims pins for all sensors."""
    try:
        h = lgpio.gpiochip_open(0)
        for sensor in SENSORS:
            lgpio.gpio_claim_output(h, sensor.pin)
        print("GPIO initialized for ultrasonic sensors.")
        return h
    except Exception as e:
        print(f"GPIO initialization failed: {e}")
        return None

def cleanup(h: int, spike_controller: SpikeController):
    """Releases all GPIO and serial resources."""
    global running
    if not running: return
    print("\nCleaning up resources...")
    running = False
    spike_controller.disconnect()
    if h is not None:
        lgpio.gpiochip_close(h)
        print("GPIO resources released.")
    print("Exiting.")

def main():
    """Main program entry point."""
    h = None
    spike_controller = SpikeController(SPIKE_PORT, SPIKE_BAUD)

    # Graceful exit handler
    def signal_handler(sig, frame):
        cleanup(h, spike_controller)
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

    # Initialization
    h = setup_gpio()
    if h is None or not spike_controller.connect():
        cleanup(h, spike_controller)
        return

    # Start sensor threads
    for sensor in SENSORS:
        thread = threading.Thread(target=sensor_worker, args=(h, sensor), daemon=True)
        thread.start()

    print("Robot starting main loop. Press Ctrl+C to stop.")
    
    # --- Main Robot Control Loop ---
    try:
        front_sensor = next(s for s in SENSORS if s.name == "frente")
        target_distance = 15.0 # cm

        while running:
            # Drain queue, we only need the latest state
            while not sensor_queue.empty():
                sensor_queue.get()

            dist = front_sensor.last_reading
            
            if dist is None or dist > 100:
                # If no obstacle or too far, move forward
                spike_controller.advance(vel=0.6, ref=0, kp=0.4, kd=0.8)
                print(f"\rSearching... Current dist: ---      ", end="")
            elif dist > target_distance:
                # Obstacle detected, approach it
                spike_controller.advance(vel=0.4, ref=0, kp=0.4, kd=0.8)
                print(f"\rApproaching... Current dist: {dist:.1f} cm", end="")
            else:
                # Reached target distance, stop
                spike_controller.stop()
                print(f"\rTarget reached! Current dist: {dist:.1f} cm ", end="")

            time.sleep(0.05) # 20Hz control loop

    finally:
        cleanup(h, spike_controller)

if __name__ == "__main__":
    main()
