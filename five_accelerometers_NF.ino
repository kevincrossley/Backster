#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include "Wire.h"
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

#define TCAADDR 0x70


/**
 * TCA9548 I2CScanner.pde -- I2C bus scanner for Arduino
 *
 * Based on code c. 2009, Tod E. Kurt, http://todbot.com/blog/
 *
 */
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   
   Connections (For default I2C)
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 5V DC
   Connect GROUND to common ground

   History
   =======
   2014/JULY/25  - First version (KTOWN)
*/
   
/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm0 = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
Adafruit_LSM9DS0 lsm1 = Adafruit_LSM9DS0(1100);
Adafruit_LSM9DS0 lsm2 = Adafruit_LSM9DS0(1200);
Adafruit_LSM9DS0 lsm3 = Adafruit_LSM9DS0(1300);
Adafruit_LSM9DS0 lsm4 = Adafruit_LSM9DS0(1400);

/* Or, use Hardware SPI:
  SCK -> SPI CLK
  SDA -> SPI MOSI
  G_SDO + XM_SDO -> tied together to SPI MISO
  then select any two pins for the two CS lines:
*/

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

/* Or, use Software SPI:
  G_SDO + XM_SDO -> tied together to the MISO pin!
  then select any pins for the SPI lines, and the two CS pins above
*/

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_SCLK, LSM9DS0_MISO, LSM9DS0_MOSI, LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(Adafruit_LSM9DS0 lsm)
{
  sensor_t accel, mag, gyro, temp;
  
  lsm.getSensor(&accel, &mag, &gyro, &temp);
  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  delay(50);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(Adafruit_LSM9DS0 lsm)
{

  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);

}


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
   while (!Serial);
    delay(1000);

    Wire.begin();
    
    Serial.begin(9600);
    Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;
        
        uint8_t data = 1;
        
        if (! twi_writeTo(addr, &data, 0, 1, 1)) {
        
           Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
    Serial.println("\ndone");
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");


  /* Initialise the sensor */
     lsm0.begin();
     lsm1.begin();
     lsm2.begin();
     lsm3.begin();
     lsm4.begin();
     
  if(!lsm0.begin())
  {
    //There was a problem detecting the LSM9DS0 ... check your connections 
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1){};
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  if(!lsm1.begin())
  {
    /*There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1){};
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
    if(!lsm2.begin())
  {
    //There was a problem detecting the LSM9DS0 ... check your connections 
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1){};
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  if(!lsm3.begin())
  {
    //There was a problem detecting the LSM9DS0 ... check your connections 
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1){};
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
    if(!lsm4.begin())
  {
    //There was a problem detecting the LSM9DS0 ... check your connections 
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1){};
  }
  Serial.println(F("Found LSM9DS0 9DOF"));

  /* Display some basic information on this sensor */
  /* I don't really like seeing all this so I commented it out
  tcaselect(7);
  displaySensorDetails(lsm0);
  tcaselect(6);
  displaySensorDetails(lsm1);
  tcaselect(5);
  displaySensorDetails(lsm2);
  tcaselect(4);
  displaySensorDetails(lsm3);
  tcaselect(3);
  displaySensorDetails(lsm4);
  */
  
  /* Setup the sensor gain and integration time */
  tcaselect(7);
  configureSensor(lsm0);
  tcaselect(6);
  configureSensor(lsm1);
  tcaselect(5);
  configureSensor(lsm2);
  tcaselect(4);
  configureSensor(lsm3);
  tcaselect(3);
  configureSensor(lsm4);
  
  /* We're ready to go! */
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
  //unsigned long tRaw;
  uint32_t tStep = 1;
  float estT = 0;

    
void loop(void) 
{  
  
  /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp;
  
  Serial.println();

  Serial.print(estT);Serial.print(",");
    
  tcaselect(7);
  lsm0.getEvent(&accel, &mag, &gyro, &temp); 
  
  // print out accelleration data
  Serial.print(accel.acceleration.x); Serial.print(",");
  Serial.print(accel.acceleration.y); Serial.print(",");
  Serial.print(accel.acceleration.z); Serial.print(",");
  
  // print out gyroscopic data
  Serial.print(gyro.gyro.x);     Serial.print(",");
  Serial.print(gyro.gyro.y);     Serial.print(",");
  Serial.print(gyro.gyro.z);     Serial.print(",");

  //print out magnetometer data
  Serial.print(mag.magnetic.x); Serial.print(",");
  Serial.print(mag.magnetic.y); Serial.print(",");
  Serial.print(mag.magnetic.z); Serial.print(",");

  tcaselect(6);
  
  lsm1.getEvent(&accel, &mag, &gyro, &temp);
   
    // print out accelleration data
  Serial.print(accel.acceleration.x); Serial.print(",");
  Serial.print(accel.acceleration.y); Serial.print(",");
  Serial.print(accel.acceleration.z); Serial.print(",");
  
  // print out gyroscopic data
  Serial.print(gyro.gyro.x);     Serial.print(",");
  Serial.print(gyro.gyro.y);     Serial.print(",");
  Serial.print(gyro.gyro.z);     Serial.print(",");

  //print out magnetometer data
  Serial.print(mag.magnetic.x); Serial.print(",");
  Serial.print(mag.magnetic.y); Serial.print(",");
  Serial.print(mag.magnetic.z); Serial.print(",");


  tcaselect(5);
  lsm2.getEvent(&accel, &mag, &gyro, &temp); 
  
    // print out accelleration data
  Serial.print(accel.acceleration.x); Serial.print(",");
  Serial.print(accel.acceleration.y); Serial.print(",");
  Serial.print(accel.acceleration.z); Serial.print(",");
  
  // print out gyroscopic data
  Serial.print(gyro.gyro.x);     Serial.print(",");
  Serial.print(gyro.gyro.y);     Serial.print(",");
  Serial.print(gyro.gyro.z);     Serial.print(",");

  //print out magnetometer data
  Serial.print(mag.magnetic.x); Serial.print(",");
  Serial.print(mag.magnetic.y); Serial.print(",");
  Serial.print(mag.magnetic.z); Serial.print(",");

  tcaselect(4);
  lsm3.getEvent(&accel, &mag, &gyro, &temp); 
  
    // print out accelleration data
  Serial.print(accel.acceleration.x); Serial.print(",");
  Serial.print(accel.acceleration.y); Serial.print(",");
  Serial.print(accel.acceleration.z); Serial.print(",");
  
  // print out gyroscopic data
  Serial.print(gyro.gyro.x);     Serial.print(",");
  Serial.print(gyro.gyro.y);     Serial.print(",");
  Serial.print(gyro.gyro.z);     Serial.print(",");

  //print out magnetometer data
  Serial.print(mag.magnetic.x); Serial.print(",");
  Serial.print(mag.magnetic.y); Serial.print(",");
  Serial.print(mag.magnetic.z); Serial.print(",");


  tcaselect(3);
  lsm4.getEvent(&accel, &mag, &gyro, &temp); 
  
    // print out accelleration data
  Serial.print(accel.acceleration.x); Serial.print(",");
  Serial.print(accel.acceleration.y); Serial.print(",");
  Serial.print(accel.acceleration.z); Serial.print(",");
  
  // print out gyroscopic data
  Serial.print(gyro.gyro.x);     Serial.print(",");
  Serial.print(gyro.gyro.y);     Serial.print(",");
  Serial.print(gyro.gyro.z);     Serial.print(",");

  //print out magnetometer data
  Serial.print(mag.magnetic.x); Serial.print(",");
  Serial.print(mag.magnetic.y); Serial.print(",");
  Serial.print(mag.magnetic.z); Serial.println(" ");

  //tRaw = millis();
  //Serial.print("Elapsed Time (s), ");Serial.print(tRaw/1000);
  

  tStep = tStep + 1;
  estT = estT + .05;
  delay(50);
}

