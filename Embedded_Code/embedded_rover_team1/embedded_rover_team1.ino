//Libraries
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>  //
#include <Adafruit_BNO055.h>  //
#include <utility/imumaths.h>
#include <SD.h>
#include <Servo.h>

// #define BMP_SCK 13
// #define BMP_MISO 12
// #define BMP_MOSI 11
// #define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

//Adafruit_BMP3XX bmp(BMP_CS); // hardware SPI
//Adafruit_BMP3XX bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);  // Software SPI

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55);


Servo Claw;
Servo Wrist;
Servo UpperElbow;
Servo LowerElbow;

// struct ros2Input{
//   bool ConnectionTest; //Testing Connection
//   bool circleButton;   //Circle Button -- Open Claw
//   bool crossButton;    //Cross Button -- Close Claw
//   bool l1Shoulder;     //L1 Left Shoulder Button -- Turn Wrist Left
//   bool r1Shoulder;     //R1 Right Shoulder Button -- Turn Wrist Right
//   // float arrowKeyLeft;       //Left Stick Y-axis -- Upper Elbow
//   // float arrowKeyRight;       //Right Stick Y-axis -- Lower Elbow
//   bool dPadLeft;       //D-pad Left Button -- Turn Turret Left
//   bool dPadRight;      // D-pad Right Button -- Turn Turret Left

//   bool arrowKeyLeft;
//   bool arrowKeyRight;
// };
// char buffer[sizeof(ros2Input)];

// struct picoOutput{
//   float lctrly;
//   float rcltry;
//   float ltrigger;
//   float rtrigger;
//   float temperature;
//   float pressure;
//   float acceleration;
//   int  servoSpeed;
//   bool lshoulder;
//   bool rshoulder;
//   bool start;
// };
// char buffer2[sizeof(picoOutput)];



// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address

// const int chipSelect = 17;

// const int m1_AIN1 = 18;
// const int m1_AIN2 = 19;
// const int m2_AIN1 = 20;
// const int m2_AIN2 = 21;

void setup () {
  Serial.begin(115200);
  while (!Serial) delay(10);  // wait for serial port to open!

  LowerElbow.attach(12);
  //Serial.println("Orientation Sensor Test"); Serial.println("");

  //Initialise the BNO
  //Serial.println("Initializing Adafruit BNO55");
  if (!bno.begin()){
    //Serial.print("Ooops, no BNO055 detected... Check your wiring.");
    while (1); // ask Jack what this does
  }
  else {
    //Serial.println("Adafruit BNO55 Initialized");
  }
  bno.setExtCrystalUse(true);

  // Initialise the BMP
  //Serial.println("Initializing Adafruit BMP388");
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    //Serial.println("Could not find a valid BMP388 sensor... Check your wiring.");
    while (1); // ask Jack what this does
  }
  else {
    //Serial.println("Adafruit BMP388 Initialized");
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  //BMP Code End

  //SD Card Begin
  // Serial.print("Initializing SD card...");
  // while (!SD.begin(chipSelect)) {
  //   Serial.println("Failed to initialize SD Card.");
  // }
  // Serial.println("SD Card itilialized successfully.");
  //SD Card End


  delay(2000);

  //SD Card End

}

void loop () {
  if (Serial.available()) {
    
    // if (mainInput.ConnectionTest == 1) {
    //     delay(500);
    //     digitalWrite(LED_BUILTIN, LOW);
    //     delay(500);
    //     digitalWrite(LED_BUILTIN, HIGH);
    // }
    if (! bmp.performReading() > 0) {
    //Serial.println("Failed to perform reading BMP #1 :(");
    return;
    }
    
    
    bmp.temperature; 
    bmp.pressure / 100.0;  
    bmp.readAltitude(SEALEVELPRESSURE_HPA);
    
    //   //Read Approximate Altitude in Meters
    // bmp.readPressure() //Reads pressure in Pascal and calculates into Hectopascal
    // bmp.readTemperature()


    //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
    

    
      
      // digitalWrite(m1_AIN1, HIGH);
      // digitalWrite(m1_AIN2, LOW);

      // delay(2500);

      // digitalWrite(m1_AIN2, HIGH);
      // digitalWrite(m1_AIN1, LOW);


      // 101.32
      // 99.76
  }
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
      const int commandSize = command.length() +1; //Command size is equal to length of command string +1
      char commands[commandSize]; 
      command.toCharArray(commands,commandSize); //create commands array using commmands and commandSize
      int LowerElbowMove = command[0]; //Acess first value in array (corresponds to axis 1)
      LowerElbow.write(LowerElbowMove); //Move axis 1
      Serial.print(LowerElbowMove);
    }
  }
  else{
    // If not print no serial avaliable
    Serial.println("No serial avaliable");
  }
}
