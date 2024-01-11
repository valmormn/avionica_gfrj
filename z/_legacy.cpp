#include "GY521.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//  Sensor de Pressão Atmosférica GY-BMP280

//  Acelerômetro e giroscópio MPU6050

//  GPS module GY-GPS6MV2 NEO-6M
GY521 sensor(0x68);

void gps_setup() {
  while (sensor.wakeup() == false)
  {
    Serial.print(millis());
    Serial.println("\tCould not connect to GY521");
    delay(1000);
  }
  sensor.setAccelSensitivity(2);  //  8g
  sensor.setGyroSensitivity(1);   //  500 degrees/s

  sensor.setThrottle();
  Serial.println("start...");

  //  set calibration values from calibration sketch.
  sensor.axe = 0.574;
  sensor.aye = -0.002;
  sensor.aze = -1.43;
  sensor.gxe = 10.702;
  sensor.gye = -6.436;
  sensor.gze = -0.676;
}

void gps_loop() {
  sensor.read();
  float x = sensor.getAngleX();
  float y = sensor.getAngleY();
  float z = sensor.getAngleZ();

  if (counter % 10 == 0)
  {
    //  Serial.println("\nCNT\tX\tY\tZ");
  }

  // Serial.print(counter);
  // Serial.print('\t');
  Serial.print(x, 1);
  Serial.print('\t');
  Serial.print(y, 1);
  Serial.print('\t');
  Serial.print(z, 1);
  Serial.println();

  counter++;
}



void setup()
{
  Serial.begin(9600);
  Serial.println();
  Serial.println(__FILE__);
  Serial.print("GY521_LIB_VERSION: ");
  Serial.println(GY521_LIB_VERSION);

  Wire.begin(); //  Inicia a comunicação I2C como controlador

  delay(100);
}


void loop()
{
  sensor.read();
  float x = sensor.getAngleX();
  float y = sensor.getAngleY();
  float z = sensor.getAngleZ();

  if (counter % 10 == 0)
  {
    //  Serial.println("\nCNT\tX\tY\tZ");
  }

  // Serial.print(counter);
  // Serial.print('\t');
  Serial.print(x, 1);
  Serial.print('\t');
  Serial.print(y, 1);
  Serial.print('\t');
  Serial.print(z, 1);
  Serial.println();

  counter++;
}
