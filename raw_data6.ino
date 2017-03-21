#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
extern "C" { 
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}


Adafruit_BNO055 bno = Adafruit_BNO055(55);

// MUX Board Connections
//                   0  1  2  3  4  5  6  7
boolean muxIMUs[] = {0, 0, 0, 0, 0, 0, 0, 0};

#define TCAADDR 0x70



void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}



void initSensorNEW(void)
{
 if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
}



/**************************************************************************/
/*                    OUR IMPORTANT FUNCTION IS HERE                      */
/**************************************************************************/
void getSensorEventNEW(int senseNum)
{

   /* The processing sketch expects data as roll, pitch, heading */
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print(quat.w(), 4);
  Serial.print(" ");
  Serial.print(quat.x(), 4);
  Serial.print(" ");
  Serial.print(quat.y(), 4);
  Serial.print(" ");
  Serial.print(quat.z(), 4);
  Serial.print(" ");
 
}




void setup(void)
{
  while (!Serial);
  delay(1000);
  Wire.begin();
  Serial.begin(9600);


     for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      //Serial.print("TCA Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;
      
        uint8_t data;
        if (! twi_writeTo(addr, &data, 0, 1, 1)) {
           //Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
           muxIMUs[t] = 1;
        }
      }
    }

  /* Initialise the sensor */
  for (int i = 0; i < sizeof(muxIMUs); i++)
  {
    if(muxIMUs[i])
    {
      //Serial.print("SensorInit ");Serial.println(i);
      tcaselect(i);
      initSensorNEW();
      //delay(1000);
    }
  }

}



 int step = 100; //time in ms that we want to pull data rate
 unsigned long time = 0;
 unsigned long prevTime;
 
void loop(void)
{
  prevTime = time;
  time = millis();
  while(time <= (prevTime + step))
  {
    time = millis();
  }
 
  Serial.print(time); Serial.print(" ");


  for (int i = 0; i < sizeof(muxIMUs); i++)
  {
    if(muxIMUs[i])
    {
      //Serial.print("Sensor ");Serial.print(i); Serial.print(", ");
      tcaselect(i);
      getSensorEventNEW(i);
    }
  }
/*  if (digitalRead(9) == LOW), 
      Serial.print(0);
  if (digitalRead(9) == HIGH)
      Serial.print(1);*/
   Serial.print("\n");
}
