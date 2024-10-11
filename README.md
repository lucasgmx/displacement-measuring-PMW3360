# Displacement Measuring with PMW3360
This project provides a displacement measuring system using the PMW3360 optical sensor. The purpose is to fine-tune the sensor to accurately measure movement in terms of X and Y coordinates. The firmware and SPI communication are utilized to interface with the sensor, capture motion data, and calculate displacement in millimeters.

## Features
- Fine-tuned PMW3360 optical sensor for accurate displacement measurement.
- SPI communication with custom firmware upload for the sensor.
- Real-time X and Y coordinate updates printed to the serial monitor.
- Calibration mechanism to adjust the sensor's output to mm units.

## Getting Started

### Hardware Requirements
- Arduino board (tested with Arduino Mega).
- PMW3360 Optical Sensor.
- SPI communication setup (MISO, MOSI, SCK, and SS pins).

### Software Requirements
- Arduino IDE.
- PMW3360 firmware file (included as `PMW3360_SROM_04.h`).

### Wiring Setup
1. Connect the PMW3360 optical sensor to the Arduino using the following pin configuration:
- MISO: Arduino pin 50
- MOSI: Arduino pin 51
- SCK: Arduino pin 52
- SS (Slave Select): Arduino pin 53 (defined as `OPTICAL_SLAVE_PIN`)
2. Ensure that your SPI communication is set to `SPI_MODE3` and most significant bit (MSB) first.

### Usage
Once the code is uploaded and the hardware is connected:
1. Open the Serial Monitor from the Arduino IDE.
2. The X and Y coordinates, representing the displacement in mm from the starting position, will be continuously printed to the monitor.

## Key Functions
- `PMW3360_begin()`: Starts communication with the PMW3360 sensor.
- `PMW3360_end()`: Ends communication with the sensor.
- `PMW3360_startup()`: Initializes the sensor, resets it, and uploads the firmware.
- `update_coordinates()`: Captures and updates the X and Y coordinates by reading sensor data and converting it to millimeters.
- `PMW3360_write_reg()`: Writes data to a specified PMW3360 register.
- `PMW3360_read_reg()`: Reads data from a specified PMW3360 register.

## Calibration
The `OPTICAL_CALIBRATION_RATIO` is set to `43.89`, calculated based on the sensor's DPI resolution. Adjust this value depending on your specific DPI setting for accurate displacement measurements.
