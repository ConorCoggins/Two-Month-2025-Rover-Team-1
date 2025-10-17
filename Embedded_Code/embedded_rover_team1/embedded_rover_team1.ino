//Libraries
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <Stepper.h>
#include <Servo.h>

Servo servo_25kg_turret_PWM_S1;

void setup () {
  Serial.begin(9600);
  Serial.print("Testing the Servo \n")

  servo_25kg_turret_PWM_S1.attach()

}

void loop () {
  //SDA_Sensor
  
  
  //SCL_Sensor
  

}