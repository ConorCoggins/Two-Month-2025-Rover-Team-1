#include <Arduino.h>
#include <Servo.h>

//Initialize servo
Servo servo1;

void setup() {
  // put your setup code here, to run once:
  // Serial initilization at a baud rate of 9600
  Serial.begin(9600);
  servo1.attach(12);
}

void readSerial(){
  // Checks if serial is avaliable (value greater than 0)
  if(Serial.available() >0){
    String command = Serial.readStringUntil('\n'); //Read serial until newline
    command.trim(); //Remove whitespace
    // If no command is received
    if(command.length() == 0){
      Serial.println("No motor command received");
      return;
    }
    else{
      // Initilize array of the size of the serial input string. Print out over serial the value that the axis was commanded to move
      int firstCommaIndex = command.indexOf(',');
      String sValue1 = command.substring(0, firstCommaIndex);
      //code start
      if(sValue1 == "moveaxis1"){ // Code start
        int secondCommaIndex = command.indexOf(',', firstCommaIndex + 1);
        String sValue2 = command.substring(firstCommaIndex + 1, secondCommaIndex);
        int value2 = sValue2.toInt();
        servo1.writeMicroseconds(value2); //Move axis 1
        Serial.print(value2);
      } // Code end

      if(sValue1 == "moveaxis2"){ // Code start
        int secondCommaIndex = command.indexOf(',', firstCommaIndex + 1); //Dotn change
        String sValue2 = command.substring(firstCommaIndex + 1, secondCommaIndex);// DOnt change
        int value2 = sValue2.toInt(); //
        servo2.writeMicroseconds(value2); //change servo name
        Serial.print(value2); //dont change
      }
    }
  }
  // else{
  //   // If not print no serial avaliable
  //   Serial.println("No serial avaliable");
  // }
}

void loop() {
  // put your main code here, to run repeatedly:
  readSerial(); //Call the read serial function
}