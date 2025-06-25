#!/usr/bin/env python3

import sys
import time
import lgpio
from typing import Optional, Tuple
from dataclasses import dataclass

# Performance tuning parameters
MIN_MEASURE_INTERVAL = 0.02  # 50Hz max update rate
SENSOR_TIMEOUT = 0.05       # 50ms timeout for echo pulse
SPEED_OF_SOUND = 34300 / 1000000  # cm/s to cm/us
STATS_WINDOW = 20           # Number of readings for statistics

@dataclass
class SensorStats:
    min_distance: float = float('inf')
    max_distance: float = 0.0
    avg_distance: float = 0.0
    read_count: int = 0
    error_count: int = 0
    last_read_time: float = 0.0

class UltrasonicSensor:
    def __init__(self, pin: int):
        self.pin = pin
        self.h = None
        self.stats = SensorStats()
        self.last_distance = None
        self.last_read_time = 0
        
    def open(self):
        """Initialize GPIO for the sensor"""
        self.h = lgpio.gpiochip_open(0)
        
    def close(self):
        """Clean up GPIO resources"""
        if self.h is not None:
            lgpio.gpiochip_close(self.h)
            self.h = None
    
    def measure(self) -> Tuple[Optional[float], float]:
        """
        Measure distance with optimized timing
        Returns: (distance in cm, measurement time in seconds)
        """
        if self.h is None:
            return None, 0
            
        start_time = time.time()
        
        # Check if we need to wait before next measurement
        time_since_last = start_time - self.last_read_time
        if time_since_last < MIN_MEASURE_INTERVAL:
            time.sleep(MIN_MEASURE_INTERVAL - time_since_last)
        
        try:
            # Send trigger pulse (minimal timing)
            lgpio.gpio_claim_output(self.h, self.pin)
            lgpio.gpio_write(self.h, self.pin, 0)
            time.sleep(0.000001)  # 1us (minimum)
            lgpio.gpio_write(self.h, self.pin, 1)
            time.sleep(0.000010)  # 10us (HC-SR04 spec)
            lgpio.gpio_write(self.h, self.pin, 0)
            
            # Switch to input mode for echo
            lgpio.gpio_claim_input(self.h, self.pin)

            # Wait for pulse start with timeout
            timeout = time.time() + SENSOR_TIMEOUT
            while lgpio.gpio_read(self.h, self.pin) == 0:
                if time.time() > timeout:
                    return None, 0

            pulse_start = time.time()
            
            # Wait for pulse end with timeout
            timeout = time.time() + SENSOR_TIMEOUT
            while lgpio.gpio_read(self.h, self.pin) == 1:
                if time.time() > timeout:
                    return None, 0
            
            pulse_end = time.time()
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            distance = (pulse_duration * SPEED_OF_SOUND * 1000000) / 2
            
            # Update statistics
            self._update_stats(distance)
            self.last_distance = distance
            self.last_read_time = time.time()
            
            return distance, pulse_duration
            
        except Exception as e:
            self.stats.error_count += 1
            return None, 0
    
    def _update_stats(self, distance: float):
        """Update running statistics"""
        if distance is None:
            self.stats.error_count += 1
            return
            
        self.stats.min_distance = min(self.stats.min_distance, distance)
        self.stats.max_distance = max(self.stats.max_distance, distance)
        
        # Running average
        total = self.stats.avg_distance * self.stats.read_count + distance
        self.stats.read_count += 1
        self.stats.avg_distance = total / self.stats.read_count
        
        # Reset stats if we have enough samples
        if self.stats.read_count >= STATS_WINDOW * 2:
            self.stats.min_distance = distance
            self.stats.max_distance = distance
            self.stats.avg_distance = distance
            self.stats.read_count = 1

def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <gpio_pin> [measurement_interval]")
        print(f"Example: {sys.argv[0]} 13 0.05")
        sys.exit(1)

    gpio_pin = int(sys.argv[1])
    measure_interval = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
    
    sensor = UltrasonicSensor(gpio_pin)
    
    try:
        sensor.open()
        print(f"Ultrasonic Sensor on GPIO{gpio_pin}")
        print("Press Ctrl+C to exit\n")
        print("Distance (cm) | Min    | Max    | Avg    | Errors")
        print("-" * 50)
        
        while True:
            start_time = time.time()
            
            # Take measurement
            distance, measure_time = sensor.measure()
            
            # Display results
            if distance is not None:
                stats = sensor.stats
                print(f"\r{distance:7.2f} cm | "
                      f"{stats.min_distance:6.1f} | "
                      f"{stats.max_distance:6.1f} | "
                      f"{stats.avg_distance:6.1f} | "
                      f"{stats.error_count:3d} errors", 
                      end='', flush=True)
            
            # Calculate sleep time to maintain desired rate
            elapsed = time.time() - start_time
            sleep_time = max(0, measure_interval - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\nMeasurement stopped by user.")
    finally:
        sensor.close()
        print("\nGPIO resources released.")

if __name__ == '__main__':
    main()
