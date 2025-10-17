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
Servo servo_55g_wrist_PWM_S2;

void setup () {
  Serial.begin(9600);
  Serial.print("Testing the Servos \n");

  //Servo 1
  // servo_25kg_turret_PWM_S1.attach(12);

  // Serial.print(" - Moving Servo 2 into Central Position \n");
  // servo_25kg_turret_PWM_S1.writeMicroseconds(1500);

  //Servo 2 
  servo_55g_wrist_PWM_S2.attach(13);

  Serial.print(" - Moving Servo 2 into Central Position \n");
  

}

void loop () {


  servo_55g_wrist_PWM_S2.write(100);
  delay(200);
  servo_55g_wrist_PWM_S2.write(0);
  delay(200);
  // servo_55g_wrist_PWM_S2.write(2);
  // delay(200);
  // servo_55g_wrist_PWM_S2.write(3);
  // delay(200);
  // servo_55g_wrist_PWM_S2.write(4);
  // delay(200);
  //SDA_Sensor
  
  
  //SCL_Sensor
  

}