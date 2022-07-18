#include <SPI.h>
#include <avr/pgmspace.h>
#include "digitalWriteFast.h"
#include "PMW3360_SROM_04.h"

// Registers
#define REG_Product_ID  0x00
#define REG_Revision_ID 0x01
#define REG_Motion  0x02
#define REG_Delta_X_L 0x03
#define REG_Delta_X_H 0x04
#define REG_Delta_Y_L 0x05
#define REG_Delta_Y_H 0x06
#define REG_SQUAL 0x07
#define REG_Raw_Data_Sum  0x08
#define REG_Maximum_Raw_data  0x09
#define REG_Minimum_Raw_data  0x0A
#define REG_Shutter_Lower 0x0B
#define REG_Shutter_Upper 0x0C
#define REG_Control 0x0D
#define REG_Config1 0x0F
#define REG_Config2 0x10
#define REG_Angle_Tune  0x11
#define REG_Frame_Capture 0x12
#define REG_SROM_Enable 0x13
#define REG_Run_Downshift 0x14
#define REG_Rest1_Rate_Lower  0x15
#define REG_Rest1_Rate_Upper  0x16
#define REG_Rest1_Downshift 0x17
#define REG_Rest2_Rate_Lower  0x18
#define REG_Rest2_Rate_Upper  0x19
#define REG_Rest2_Downshift 0x1A
#define REG_Rest3_Rate_Lower  0x1B
#define REG_Rest3_Rate_Upper  0x1C
#define REG_Observation 0x24
#define REG_Data_Out_Lower  0x25
#define REG_Data_Out_Upper  0x26
#define REG_Raw_Data_Dump 0x29
#define REG_SROM_ID 0x2A
#define REG_Min_SQ_Run  0x2B
#define REG_Raw_Data_Threshold  0x2C
#define REG_Config5 0x2F
#define REG_Power_Up_Reset  0x3A
#define REG_Shutdown  0x3B
#define REG_Inverse_Product_ID  0x3F
#define REG_LiftCutoff_Tune3  0x41
#define REG_Angle_Snap  0x42
#define REG_LiftCutoff_Tune1  0x4A
#define REG_Motion_Burst  0x50
#define REG_LiftCutoff_Tune_Timeout 0x58
#define REG_LiftCutoff_Tune_Min_Length  0x5A
#define REG_SROM_Load_Burst 0x62
#define REG_Lift_Config 0x63
#define REG_Raw_Data_Burst  0x64
#define REG_LiftCutoff_Tune2  0x65

#define OPTICAL_SLAVE_PIN 53   // Slave Select pin for optical sensor

#define OPTICAL_CALIBRATION_RATIO 43.89 //39.37 //70.86 // Ratio = (resolution/25.4mm) // example: the ratio for 1000 dpi will be (1000/25.4) = 39.37.

// FIRMWARE "PMW3360_SROM_04.h"
extern const unsigned short firmware_length;
extern const unsigned char firmware_data[];

float sensor_coordinates[2] = {0,0};  // coordinates X, Y in mm
volatile byte xy_data[4];              // actual sensor reading (X low, X high, Y low, Y high)
int16_t * x = (int16_t *) &xy_data[0]; // combine lower and higher bytes of Delta_X together
int16_t * y = (int16_t *) &xy_data[2]; // combine lower and higher bytes of Delta_Y together

void setup()
{
  Serial.begin(115200);

  pinMode(OPTICAL_SLAVE_PIN, OUTPUT);

  PMW3360_end(); // end communitation with the devices

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setBitOrder(MSBFIRST); // most significant bit first
  SPI.setClockDivider(8);

  PMW3360_startup(); // startup the PMW3360 motion sensors
  delay(100);

  PMW3360_write_reg(REG_Config1, 0x0A); // 1000dpi
}

void PMW3360_begin() // start communication with the devices
{
  digitalWriteFast(OPTICAL_SLAVE_PIN, LOW);
}

void PMW3360_end() // end communitation with the devices
{
  digitalWriteFast(OPTICAL_SLAVE_PIN, HIGH);
}

void PMW3360_startup() // startup the devices
{
  PMW3360_end();   // end communitation with the devices
  PMW3360_begin(); // start communication with the devices
  PMW3360_end();   // end communitation with the devices
  PMW3360_write_reg(REG_Power_Up_Reset, 0x5A); // force reset
  delay(50); // wait for it to reboot
  // read registers 0x02 to 0x06 (and discard the data)
  PMW3360_read_reg(REG_Motion);
  PMW3360_read_reg(REG_Delta_X_L);
  PMW3360_read_reg(REG_Delta_X_H);
  PMW3360_read_reg(REG_Delta_Y_L);
  PMW3360_read_reg(REG_Delta_Y_H);
  // upload the firmware
  PMW3360_upload_firmware();
  delay(10);
  // make the X/Y registers available for reading
  PMW3360_write_reg(REG_Motion, 0x01);
}

void PMW3360_upload_firmware() // upload the firmware to the sensors
{
  //Write 0 to Rest_En bit of Config2 register to disable Rest mode.
  PMW3360_write_reg(REG_Config2, 0x20);
  // write 0x1d in SROM_enable reg for initializing
  PMW3360_write_reg(REG_SROM_Enable, 0x1D);
  // wait for more than one frame period
  delay(10); // assume that the frame rate is as low as 100fps
  // write 0x18 to SROM_enable to start SROM download
  PMW3360_write_reg(REG_SROM_Enable, 0x18);
  // write the SROM file (=firmware data)
  PMW3360_begin(); // start communication with the devices
  SPI.transfer(REG_SROM_Load_Burst | 0x80); // write burst destination adress
  delayMicroseconds(15);
  // send all bytes of the firmware
  unsigned char c;
  for (int i = 0; i < firmware_length; i++)
  {
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    SPI.transfer(c);
    delayMicroseconds(15);
  }
  PMW3360_end();   // end communitation with the devices
}

void PMW3360_write_reg(byte reg_addr, byte data) // write byte to register
{
  PMW3360_begin(); // start communication with the devices
  //send adress of the register, with MSBit = 1 to indicate it's a write
  SPI.transfer(reg_addr | 0x80 );
  //sent data
  SPI.transfer(data);
  delayMicroseconds(20); // tSCLK-NCS for write operation
  PMW3360_end();   // end communitation with the devices
  delayMicroseconds(100); // tSWW/tSWR (=120us) minus tSCLK-NCS. Could be shortened,
  // but it looks like a safe lower bound
}

byte PMW3360_read_reg(byte reg_addr) // read byte from register
{
  PMW3360_begin(); // start communication with the devices
  // send adress of the register, with MSBit = 0 to indicate it's a read
  SPI.transfer(reg_addr & 0x7F );
  delayMicroseconds(100); // tSRAD
  // read data
  byte data = SPI.transfer(0);
  delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
  PMW3360_end();   // end communitation with the devices
  delayMicroseconds(19); // tSRW/tSRR (=20us) minus tSCLK-NCS
  return data;
}

void update_coordinates() // update the variable sensor_coordinates
{
  PMW3360_read_reg(REG_Motion); // read the Motion register and discard the data, just to freeze the X/Y registers
  for (int x = 0; x <= 3; x++)
  {
    digitalWriteFast(OPTICAL_SLAVE_PIN, LOW); // start communication with the device
    SPI.transfer((x + 0x03) & 0x7F ); // read registers from 0x03 to 0x06
    delayMicroseconds(100); // tSRAD
    xy_data[x] = SPI.transfer(0);
    delayMicroseconds(1); // tSCLK-NCS for read operation is 120ns
    digitalWriteFast(OPTICAL_SLAVE_PIN, HIGH); // end communitation with the device
    delayMicroseconds(19); // tSRW/tSRR (=20us) minus tSCLK-NCS
  }
  sensor_coordinates[0] += float(*x) / OPTICAL_CALIBRATION_RATIO; //calculate the travel distance in X in mm
  sensor_coordinates[1] -= float(*y) / OPTICAL_CALIBRATION_RATIO; //calculate the travel distance in Y in mm
}

void loop()
{
  update_coordinates();

  Serial.print("X: ");
  Serial.print(sensor_coordinates[0],5);
  Serial.print("      Y: ");
  Serial.println(sensor_coordinates[1],5);
}
