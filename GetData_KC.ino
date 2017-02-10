#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}

   
/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

// MUX Board Connections
//                   0  1  2  3  4  5  6  7
boolean muxIMUs[] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long time;

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);
#define TCAADDR 0x70

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_SCLK, LSM9DS0_MISO, LSM9DS0_MOSI, LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);


void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void displaySensorDetails(void)
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
  
  delay(500);
}


void configureSensor(void)
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


void initSensorNEW(void)
{
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Display some basic information on this sensor */
  //displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  //configureSensor();
  
  /* We're ready to go! */
  Serial.println("");
}

/**************************************************************************/
/**************************************************************************/
/*
 * 
    OUR IMPORTANT FUNCTION IS HERE
 *   
*/
/**************************************************************************/
/**************************************************************************/

void getSensorEventNEW(int senseNum)
{
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 

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
  Serial.print(mag.magnetic.z); Serial.print(", ");

  //delay(10);
}


void setup(void) 
{
    while (!Serial);
    delay(1000);

    Wire.begin();
    
    Serial.begin(115200);
    Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;
      
        uint8_t data;
        if (! twi_writeTo(addr, &data, 0, 1, 1)) {
           Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
           muxIMUs[t] = 1;
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
  for (int i = 0; i < sizeof(muxIMUs); i++)
  {
    if(muxIMUs[i])
    {
      Serial.print("SensorInit ");Serial.println(i);
      tcaselect(i);
      initSensorNEW();
      //delay(1000);
    }
  }
}


void loop(void) 
{  
  // print time since program started
  time = millis();
  //double timeS = time/1000;
  Serial.print(time); Serial.print(",");
  
  for (int i = 0; i < sizeof(muxIMUs); i++)
  {
    if(muxIMUs[i])
    {
      //Serial.print("Sensor ");Serial.print(i); Serial.print(", ");
      tcaselect(i);
      getSensorEventNEW(i);
      
    }
  }

  /* this print line separates one set of data from another */
  Serial.println("\n");
  /* This delay is the break between sets (one set is collecting data from all 5 sensors one time)
  with no delays, the serial monitor prints one set of data in 0.06 seconds */
  delay(100);
}
