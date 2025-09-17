# Ultrasonic Sensor for Raspberry Pi 5

This project provides a Python script to measure distance using an HC-SR04 ultrasonic sensor with a Raspberry Pi 5, using the `lgpio` library for reliable GPIO control.

## Prerequisites

- Raspberry Pi 5 (tested on Bookworm)
- HC-SR04 Ultrasonic Sensor
- Python 3.x
- lgpio library

## Wiring

Connect the HC-SR04 sensor to your Raspberry Pi 5 as follows:

- VCC to 5V (pin 2)
- TRIG to GPIO5 (pin 29)
- ECHO to GPIO6 (pin 31)
- GND to GND (pin 39)

## Installation

1. Install the required package:
   ```bash
   sudo apt update
   sudo apt install python3-lgpio -y
   ```

## Usage

1. Navigate to the script directory:
   ```bash
   cd /home/maker/spike/ultrasonico
   ```

2. Make the script executable:
   ```bash
   chmod +x ultrasonic_lgpio.py
   ```

3. Run the script with sudo (required for GPIO access):
   ```bash
   sudo ./ultrasonic_lgpio.py [trigger_pin] [echo_pin]
   ```
   
   Example (using default pins 5 and 6):
   ```bash
   sudo ./ultrasonic_lgpio.py
   ```
   
   Example (with custom pins 17 and 27):
   ```bash
   sudo ./ultrasonic_lgpio.py 17 27
   ```

4. The script will display distance measurements in centimeters. Press `Ctrl+C` to stop.

## Troubleshooting

1. If you get permission errors, ensure you're running with `sudo`.
2. If you see inconsistent readings, check your wiring and ensure the sensor has a stable power supply.
3. Make sure no other processes are using the GPIO pins.

## License

This project is open source and available under the MIT License.
