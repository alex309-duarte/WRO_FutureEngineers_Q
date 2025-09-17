#!/usr/bin/env python3

import sys
import time
import lgpio

if len(sys.argv) < 2:
    print(f"Usage: {sys.argv[0]} <gpio_pin> [update_hz]")
    print(f"Example: {sys.argv[0]} 13 50  # 50Hz update rate")
    sys.exit(1)

GPIO_PIN = int(sys.argv[1])
UPDATE_HZ = float(sys.argv[2]) if len(sys.argv) > 2 else 50  # Default 50Hz
UPDATE_INTERVAL = 1.0 / UPDATE_HZ

# Speed of sound in cm/us
SPEED_OF_SOUND = 34300 / 1000000  # cm/s to cm/us

def get_distance(h, pin):
    """Measures distance using an ultrasonic sensor on a single pin."""
    try:
        # Send trigger pulse (minimal timing)
        lgpio.gpio_claim_output(h, pin)
        lgpio.gpio_write(h, pin, 0)
        time.sleep(0.000001)  # 1us (minimum)
        lgpio.gpio_write(h, pin, 1)
        time.sleep(0.000010)  # 10us (HC-SR04 spec)
        lgpio.gpio_write(h, pin, 0)

        # Listen for echo pulse
        lgpio.gpio_claim_input(h, pin)

        # Wait for pulse start with timeout (50ms)
        timeout = time.time() + 0.05
        while lgpio.gpio_read(h, pin) == 0:
            if time.time() > timeout:
                return None

        pulse_start = time.time()
        
        # Wait for pulse end with timeout (50ms)
        timeout = time.time() + 0.05
        while lgpio.gpio_read(h, pin) == 1:
            if time.time() > timeout:
                return None
        
        pulse_end = time.time()
        
        # Calculate distance in cm
        pulse_duration = pulse_end - pulse_start
        return (pulse_duration * SPEED_OF_SOUND * 1000000) / 2

    except Exception as e:
        print(f"Error: {e}")
        return None

def main():
    h = None
    try:
        # Open the default GPIO chip
        h = lgpio.gpiochip_open(0)
        print(f"Ultrasonic Sensor on GPIO{GPIO_PIN} at {UPDATE_HZ}Hz")
        print("Press Ctrl+C to exit\n")
        
        min_distance = float('inf')
        max_distance = 0
        read_count = 0
        error_count = 0
        
        while True:
            start_time = time.time()
            
            # Take measurement
            distance = get_distance(h, GPIO_PIN)
            
            # Update statistics
            if distance is not None:
                min_distance = min(min_distance, distance)
                max_distance = max(max_distance, distance)
                read_count += 1
                print(f"\rDistance: {distance:6.2f} cm | "
                      f"Min: {min_distance:5.1f} | "
                      f"Max: {max_distance:5.1f} | "
                      f"Reads: {read_count:4d} | "
                      f"Errors: {error_count:3d}", 
                      end='', flush=True)
            else:
                error_count += 1
            
            # Calculate sleep time to maintain desired rate
            elapsed = time.time() - start_time
            sleep_time = max(0, UPDATE_INTERVAL - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user.")
    finally:
        if h:
            lgpio.gpiochip_close(h)
        print("\nGPIO resources released.")

if __name__ == '__main__':
    main()
