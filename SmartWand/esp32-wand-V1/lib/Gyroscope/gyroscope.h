/*
* This file is to take and wrap the raw data recieved from the MPU6050 and GY-
*/

#ifndef GYROSCOPE_H
#define GYROSCOPE_H


#include <Arduino.h>
#include <Wire.h>

#define ACCEL_DEFAULT_ADR 0x68

class GYROSCOPE
{
public:
  bool begin(uint8_t address = ACCEL_DEFAULT_ADR, TwoWire &wirePort = Wire); //Begin comm with accel at given I2C address, and given wire port
  bool isConnected();                                                        //Returns true if an accel sensor is detected at library's I2C address
  bool available();                                                          //Returns true if new accel data is available
  void waitForNewData();                                                     //Block until new data is available
  bool temperatureAvailable();                                               //Returns true if new temp data is available

  float getX(); //Return latest accel data in milli-g's. If data has already be read, initiate new read.
  float getY();
  float getZ();
  int16_t getRawX(); //Return raw 16 bit accel reading
  int16_t getRawY();
  int16_t getRawZ();
  float getTemperature(); //Returns latest temp data in C. If data is old, initiate new read.

  void parseAccelData(); //Load sensor data into global vars. Call after new data is avaiable.
  void getTempData();

  void enableTemperature();  //Enable the onboard temp sensor
  void disableTemperature(); //Disable the onboard temp sensor

  void setDataRate(uint8_t dataRate); //Set the output data rate of sensor. Higher rates consume more current.
  uint8_t getDataRate();              //Returns the output data rate of sensor.

  void setScale(uint8_t scale); //Set full scale: +/-2, 4, 8, or 16g
  uint8_t getScale();           //Returns current scale of sensor

  void setMode(uint8_t mode); //Set mode to low, normal, or high data rate
  uint8_t getMode();          //Get current sensor mode

  void enableSelfTest(bool direction = true);
  void disableSelfTest();

  void enableTapDetection(); //Enable the single tap interrupt
  void disableTapDetection();
  void setTapThreshold(uint8_t threshold); //Set the 7-bit threshold value for tap and double tap
  bool isTapped();                         //Returns true if Z, Y, or X tap detection bits are set

  void setInt1Threshold(uint8_t threshold);
  uint8_t getInt1Threshold(void);
  void setInt1Duration(uint8_t duration);
  uint8_t getInt1Duration(void);

  void setIntPolarity(uint8_t level);
  void setInt1IA1(bool enable);
  void setInt1Latch(bool enable);
  void setInt1(bool enable);

  bool getInt1(void);

  lis2dh12_ctx_t dev_ctx;

  static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
  static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);

  uint8_t _i2cAddress;
  TwoWire *_i2cPort;

private:
  bool xIsFresh = false;
  bool yIsFresh = false;
  bool zIsFresh = false;
  bool tempIsFresh = false;

  uint8_t currentScale = 0; //Needed to convert readings to mg
  uint8_t currentMode = 0;  //Needed to convert readings to mg

  float accelX;
  float accelY;
  float accelZ;
  uint16_t rawX;
  uint16_t rawY;
  uint16_t rawZ;
  float temperatureC;
};


#endif //Gyroscope_h