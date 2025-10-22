//Libraries
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>  //BMP gives us
#include <Adafruit_BNO055.h>  //
#include <utility/imumaths.h>
#include <SD.h>
#include <Servo.h>

#define BMP_SCK 5
#define BMP_SDA 4

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

Servo servo_25kg_turret_PWM_S1;
Servo servo_55g_wrist_PWM_S2;
Servo servo_45kg_claw_PWM_S3;

struct ros2Input{
  bool ConnectionTest; //Testing Connection
  bool circleButton;   //Circle Button -- Open Claw
  bool crossButton;    //Cross Button -- Close Claw
  bool l1Shoulder;     //L1 Left Shoulder Button -- Turn Wrist Left
  bool r1Shoulder;     //R1 Right Shoulder Button -- Turn Wrist Right
  float lStickY;       //Left Stick Y-axis -- Upper Elbow
  float rStickY;       //Right Stick Y-axis -- Lower Elbow
  bool dPadLeft;       //D-pad Left Button -- Turn Turret Left
  bool dPadRight;      // D-pad Right Button -- Turn Turret Left
};
char buffer[sizeof(ros2Input)];

struct picoOutput{
  float lctrly;
  float rcltry;
  float ltrigger;
  float rtrigger;
  float temperature;
  float pressure;
  float acceleration;
  int  servoSpeed;
  bool lshoulder;
  bool rshoulder;
  bool start;
};
char buffer2[sizeof(picoOutput)];

/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
const int chipSelect = 17;
File dataLog;

const int m1_AIN1 = 18;
const int m1_AIN2 = 19;
const int m2_AIN1 = 20;
const int m2_AIN2 = 21;

void setup () {
  Serial.begin(9600);
  while (!Serial);
  Serial.print("Testing the Servos \n");

  //Servo 1
  servo_25kg_turret_PWM_S1.attach(12);

  Serial.print(" - Moving Servo 1 into Central Position \n");
  servo_25kg_turret_PWM_S1.writeMicroseconds(1500);

  //Servo 2 
  servo_55g_wrist_PWM_S2.attach(13);

  Serial.print(" - Moving Servo 2 into Central Position \n");
  servo_55g_wrist_PWM_S2.writeMicroseconds(1500);

  //Servo 3
  servo_45kg_claw_PWM_S3.attach(14);

  Serial.print(" - Moving Servo 3 into Central Position \n");
  servo_45kg_claw_PWM_S3.writeMicroseconds(1500);


  ////read_all_data example copy paste
  //BNO Code Begin

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }


  //BMP Code Begin
  Serial.println("Adafruit BMP388");
  if (!bmp.begin_I2C()) {
    Serial.println("Could not find a vaild BMP3 Sensor, Check Wiring.");
    while (1);
  }
  //BMP Code End

  delay(1000);


  //SD Card Begin
  while (!SD.begin(chipSelect)) {
    Serial.println("Failed to initialize SD Card.");
  }

  Serial.println("SD Card itilialized successfully.");
  //SD Card End


  delay(2000);

  //SD Card End
  ////

}

void loop () {
  

  servo_25kg_turret_PWM_S1.write(100);
  delay(2000);
  servo_25kg_turret_PWM_S1.write(0);
  delay(2000);

  servo_55g_wrist_PWM_S2.write(100);
  delay(2000);
  servo_55g_wrist_PWM_S2.write(0);
  delay(2000);

  servo_45kg_claw_PWM_S3.write(100);
  delay(2000);
  servo_45kg_claw_PWM_S3.write(0);
  delay(2000);
  
  //BNO Data Begin
  

  
  //SCL_Sensor
  

  ////read_all_data example copy paste Begin

  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&magnetometerData);
  printEvent(&accelerometerData);
  printEvent(&gravityData);

  int8_t boardTemp = bno.getTemp();
  Serial.println();
  Serial.print(F("temperature: "));
  Serial.println(boardTemp);

  uint8_t system = 0;
  uint8_t gyro = 0;
  uint8_t accel = 0;
  uint8_t mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.println();
  Serial.print("Calibration: Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.println(mag);

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
  
  digitalWrite(m1_AIN1, HIGH);
  digitalWrite(m1_AIN2, LOW);

  delay(2500);

  digitalWrite(m1_AIN2, HIGH);
  digitalWrite(m1_AIN1, LOW);
  
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);


  //BNO Data End

  //BMP Data Begin
  
  if (! bmp.performReading()) {
   Serial.println("Failed to perform reading :(");
   return;
 }
 Serial.print("Temperature = ");
 Serial.print(bmp.temperature);
  Serial.println(" *C");

 Serial.print("Pressure = ");
 Serial.print(bmp.pressure / 100.0);
 Serial.println(" hPa");

 Serial.print("Approx. Altitude = ");
 Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
 Serial.println(" m");

 Serial.println();

 // BMP Data End

  //SD Card Begin
  // File dataLog = SD.open("dataLog.txt", FILE_WRITE);

  // if (dataLog) {
  //   dataLog.print(&orientationData); // Add sensor data variables to print to SD card
  //   dataLog.print(','); 
  //   dataLog.print(&angVelocityData);
  //   dataLog.print(',');    
  //   dataLog.print(&linearAccelData);
  //   dataLog.print(',');   
  //   dataLog.print(&accelerometerData);
  //   dataLog.print(',');   
  //   dataLog.print(&gravityData);
  //   dataLog.print(',');
  //   dataLog.print(&bmp.readTemperature);
  //   dataLog.print(',');
  //   dataLog.print(&bmp.readPressure);
  //   dataLog.print(',');
  //   dataLog.print(&bmp.readAltitude);
  //   dataLog.print(',');
  //   dataLog.println();
  //   dataLog.close();// Dont add anything here
  //   Serial.println(); // Add sensor data variables you want to print to serial monitor
  // } else{
  //   Serial.println("Failed to open dataLog.txt");
  // }

  delay(2000);

  // BNO Data end

  //BMP Data Begin
  //BMP Data End


  //SD Card Begin

  //SD Card END

}