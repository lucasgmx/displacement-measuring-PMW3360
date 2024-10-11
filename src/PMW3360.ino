// Measuring distance with PMW3360 optical sensor
// Author: Lucas Marques
// Date: May 2017

#include <SPI.h>
#include <avr/pgmspace.h>
#include "digitalWriteFast.h"
#include "PMW3360_SROM_04.h"

// Optical sensor register addresses
#define REG_Product_ID 0x00
#define REG_Revision_ID 0x01
#define REG_Motion 0x02
#define REG_Delta_X_L 0x03
#define REG_Delta_X_H 0x04
#define REG_Delta_Y_L 0x05
#define REG_Delta_Y_H 0x06
#define REG_SQUAL 0x07
#define REG_Raw_Data_Sum 0x08
#define REG_Maximum_Raw_data 0x09
#define REG_Minimum_Raw_data 0x0A
#define REG_Shutter_Lower 0x0B
#define REG_Shutter_Upper 0x0C
#define REG_Control 0x0D
#define REG_Config1 0x0F
#define REG_Config2 0x10
#define REG_Angle_Tune 0x11
#define REG_Frame_Capture 0x12
#define REG_SROM_Enable 0x13
#define REG_Run_Downshift 0x14
#define REG_Rest1_Rate_Lower 0x15
#define REG_Rest1_Rate_Upper 0x16
#define REG_Rest1_Downshift 0x17
#define REG_Rest2_Rate_Lower 0x18
#define REG_Rest2_Rate_Upper 0x19
#define REG_Rest2_Downshift 0x1A
#define REG_Rest3_Rate_Lower 0x1B
#define REG_Rest3_Rate_Upper 0x1C
#define REG_Observation 0x24
#define REG_Data_Out_Lower 0x25
#define REG_Data_Out_Upper 0x26
#define REG_Raw_Data_Dump 0x29
#define REG_SROM_ID 0x2A
#define REG_Min_SQ_Run 0x2B
#define REG_Raw_Data_Threshold 0x2C
#define REG_Config5 0x2F
#define REG_Power_Up_Reset 0x3A
#define REG_Shutdown 0x3B
#define REG_Inverse_Product_ID 0x3F
#define REG_LiftCutoff_Tune3 0x41
#define REG_Angle_Snap 0x42
#define REG_LiftCutoff_Tune1 0x4A
#define REG_Motion_Burst 0x50
#define REG_LiftCutoff_Tune_Timeout 0x58
#define REG_LiftCutoff_Tune_Min_Length 0x5A
#define REG_SROM_Load_Burst 0x62
#define REG_Lift_Config 0x63
#define REG_Raw_Data_Burst 0x64
#define REG_LiftCutoff_Tune2 0x65

#define OPTICAL_SLAVE_PIN 53 // Slave Select pin for optical sensor
#define OPTICAL_CALIBRATION_RATIO 43.89 // Ratio = (resolution/25.4mm)

// FIRMWARE "PMW3360_SROM_04.h"
extern const unsigned short firmware_length;
extern const unsigned char firmware_data[];

// Measurement storage
float sensor_coordinates[2] = {0, 0};  // Coordinates X, Y in mm
volatile byte xy_data[4];               // Sensor reading (X low, X high, Y low, Y high)
int16_t *x = (int16_t *)&xy_data[0];    // Combine low and high bytes of Delta_X
int16_t *y = (int16_t *)&xy_data[2];    // Combine low and high bytes of Delta_Y

// Setup function
void setup() {
  Serial.begin(115200);
  pinMode(OPTICAL_SLAVE_PIN, OUTPUT);
  PMW3360_end();  // End communication with the devices
  
  // Initialize the optical sensor
  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST);  // MSB first
  SPI.setClockDivider(8);
  PMW3360_startup();  // Startup the PMW3360 motion sensor
  delay(100);  // Wait for sensor to stabilize
  PMW3360_write_reg(REG_Config1, 0x0A);  // Set to 1000dpi
}

// Start communication with the device
void PMW3360_begin() {
  digitalWriteFast(OPTICAL_SLAVE_PIN, LOW);
}

// End communication with the device
void PMW3360_end() {
  digitalWriteFast(OPTICAL_SLAVE_PIN, HIGH);
}

// Startup the PMW3360 device
void PMW3360_startup() {
  PMW3360_end();
  PMW3360_begin();
  PMW3360_end();
  PMW3360_write_reg(REG_Power_Up_Reset, 0x5A);  // Force reset
  delay(50);  // Wait for reset
  
  // Read and discard registers 0x02 to 0x06
  PMW3360_read_reg(REG_Motion);
  PMW3360_read_reg(REG_Delta_X_L);
  PMW3360_read_reg(REG_Delta_X_H);
  PMW3360_read_reg(REG_Delta_Y_L);
  PMW3360_read_reg(REG_Delta_Y_H);
  
  // Upload firmware
  PMW3360_upload_firmware();
  delay(10);  // Wait for SROM to be loaded
  
  // Make X/Y registers available for reading
  PMW3360_write_reg(REG_Motion, 0x01);
}

// Upload the firmware to the sensor
void PMW3360_upload_firmware() {
  PMW3360_write_reg(REG_Config2, 0x20);  // Disable Rest mode
  PMW3360_write_reg(REG_SROM_Enable, 0x1D);  // Initialize SROM
  delay(10);  // Wait for more than one frame period (assuming lowest frame rate of 100fps)
  PMW3360_write_reg(REG_SROM_Enable, 0x18);  // Start SROM download

  PMW3360_begin();  // Start communication
  SPI.transfer(REG_SROM_Load_Burst | 0x80);  // Write burst destination address
  delayMicroseconds(15);  // Ensure timing requirements for burst mode
  
  // Send firmware data
  for (int i = 0; i < firmware_length; i++) {
    SPI.transfer(pgm_read_byte(firmware_data + i));
    delayMicroseconds(15);  // Ensure timing for each byte during burst mode transfer
  }
  
  PMW3360_end();  // End communication
}

// Write a byte to a register
void PMW3360_write_reg(byte reg_addr, byte data) {
  PMW3360_begin();
  SPI.transfer(reg_addr | 0x80);  // Write address
  SPI.transfer(data);  // Send data
  delayMicroseconds(20); // tSWW/tSRR (20us total)
  PMW3360_end();
  delayMicroseconds(100);  // tSCLK-NCS for write operation (100us for 120ns requirement)
}

// Read a byte from a register
byte PMW3360_read_reg(byte reg_addr) {
  PMW3360_begin();
  SPI.transfer(reg_addr & 0x7F);  // Read address
  delayMicroseconds(100);  // tSRAD (100us total)
  byte data = SPI.transfer(0);  // Read data
  delayMicroseconds(1);  // tSCLK-NCS for read operation (1us for 120ns requirement)
  PMW3360_end();
  delayMicroseconds(19);  // tSRW/tSRR (20us total - 1us for tSCLK-NCS)
  return data;
}

// Update sensor coordinates
void update_coordinates() {
  PMW3360_read_reg(REG_Motion);  // Freeze X/Y registers
  
  // Read X and Y registers
  for (int i = 0; i <= 3; i++) {
    PMW3360_begin();
    SPI.transfer((i + 0x03) & 0x7F);
    delayMicroseconds(100);  // tSRAD (100us total)
    xy_data[i] = SPI.transfer(0);
    delayMicroseconds(1);  // tSCLK-NCS for read operation (1us for 120ns requirement)
    PMW3360_end();
    delayMicroseconds(19);  // tSRW/tSRR (20us total - 1us for tSCLK-NCS)
  }
  
  // Calculate travel distance in X and Y (in mm)
  sensor_coordinates[0] += float(*x) / OPTICAL_CALIBRATION_RATIO;
  sensor_coordinates[1] -= float(*y) / OPTICAL_CALIBRATION_RATIO;
}

// Main loop
void loop() {
  update_coordinates();  // Get new sensor reading
  
  // Print current coordinates
  Serial.print("X: ");
  Serial.print(sensor_coordinates[0], 5);
  Serial.print("    Y: ");
  Serial.println(sensor_coordinates[1], 5);
}
