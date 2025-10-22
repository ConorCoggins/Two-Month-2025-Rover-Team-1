const int _MISO = 16;
const int _MOSI = 19;
const int _CS = 17;
const int _SCK = 18;

// SDIO mode pins (set to -1 for SPI mode)
const int RP_CLK_GPIO = -1;
const int RP_CMD_GPIO = -1;
const int RP_DAT0_GPIO = -1;

// Servo pins
const int CLAW_PIN = 2;
const int WRIST_PIN = 3;
const int ELBOW1_PIN = 4;
const int ELBOW2_PIN = 5;
const int TURRET_PIN = 6;

// I2C pins (default for Pico)
const int SDA_PIN = 4;
const int SCL_PIN = 5;

#include <SPI.h>
#include <SD.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Servo.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher talker("talker", &str_msg);

ros::Subscriber<std_msgs::String> sub("commands", &commandCallback);

Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP3xx bmp;

Servo clawServo;
Servo wristServo;
Servo elbow1Servo;
Servo elbow2Servo;
Servo turretServo;

// Sensor variables (use float instead of int for precision)
float acceleration = 0.0;
float altitude = 0.0;
float temperature = 0.0;
float pressure = 0.0;

File sensorData;

void commandCallback(const std_msgs::String& cmd_msg) {
  String command = cmd_msg.data;

  if (command == "OPENCLAW") {
    clawServo.write(90);  // Open position
  } 
  else if (command == "CLOSECLAW") {
    clawServo.write(0);   // Close position
  } 
  else if (command == "WRISTUP") {
    wristServo.write(90);
  } 
  else if (command == "WRISTDOWN") {
    wristServo.write(0);
  } 
  else if (command == "ELBOW1UP") {
    elbow1Servo.write(90);
  } 
  else if (command == "ELBOW1DOWN") {
    elbow1Servo.write(0);
  } 
  else if (command == "ELBOW2UP") {
    elbow2Servo.write(90);
  } 
  else if (command == "ELBOW2DOWN") {
    elbow2Servo.write(0);
  } 
  else if (command == "TURRETRIGHT") {
    turretServo.write(90);
  } 
  else if (command == "TURRETLEFT") {
    turretServo.write(0);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Initialize sensors
  if (!bno.begin()) {
    Serial.println("BNO055 not connected");
    while (1);
  }
  
  if (!bmp.begin_I2C()) {
    Serial.println("BMP388 not connected");
    while (1);
  }

  // Initialize servos
  clawServo.attach(CLAW_PIN);
  wristServo.attach(WRIST_PIN);
  elbow1Servo.attach(ELBOW1_PIN);
  elbow2Servo.attach(ELBOW2_PIN);
  turretServo.attach(TURRET_PIN);

  // Set initial servo positions
  clawServo.write(0);
  wristServo.write(45);
  elbow1Servo.write(45);
  elbow2Servo.write(45);
  turretServo.write(45);

  // Initialize ROS
  nh.initNode();
  nh.advertise(talker);
  nh.subscribe(sub);

  // Initialize SD card
  Serial.print("Initializing SD card...");
  
  bool sdInitialized = false;
  if (RP_CLK_GPIO >= 0) {
    sdInitialized = SD.begin(RP_CLK_GPIO, RP_CMD_GPIO, RP_DAT0_GPIO);
  } else {
    if (_MISO == 0 || _MISO == 4 || _MISO == 16) {
      SPI.setRX(_MISO);
      SPI.setTX(_MOSI);
      SPI.setSCK(_SCK);
      sdInitialized = SD.begin(_CS);
    } else if (_MISO == 8 || _MISO == 12) {
      SPI1.setRX(_MISO);
      SPI1.setTX(_MOSI);
      SPI1.setSCK(_SCK);
      sdInitialized = SD.begin(_CS, SPI1);
    } else {
      Serial.println(F("ERROR: Unknown SPI Configuration"));
      return;
    }
  }

  if (!sdInitialized) {
    Serial.println("SD initialization failed!");
    return;
  }
  Serial.println("SD initialization done.");
}

void loop() {
  // Read BNO055 accelerometer data
  sensors_event_t accelerometerData;
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  acceleration = accelerometerData.acceleration.x;  // Using X-axis as example

  // Read BMP388 data
  if (bmp.performReading()) {
    temperature = bmp.temperature;
    pressure = bmp.pressure / 100.0;  // Convert Pa to hPa
    
    // Calculate altitude using standard atmosphere formula
    altitude = 44330.0 * (1.0 - pow(pressure / 1013.25, 0.1903));
  }

  // Create ROS message
  String sensorDataStr = String("ACC:") + acceleration + 
                        ",TEMP:" + temperature + 
                        ",ALT:" + altitude + 
                        ",PRES:" + pressure;
  str_msg.data = sensorDataStr;
  talker.publish(&str_msg);

  // Log to SD card
  logToSD();

  // ROS spin
  nh.spinOnce();
  delay(100);
}

// Function to log sensor data to SD card
void logToSD() {
  sensorData = SD.open("sensordata.txt", FILE_WRITE);
  if (sensorData) {
    unsigned long currentTime = millis();
    sensorData.print("Time:");
    sensorData.print(currentTime);
    sensorData.print(",ACC:");
    sensorData.print(acceleration);
    sensorData.print("ft/s^2,TEMP:");
    sensorData.print(temperature);
    sensorData.print("C,ALT:");
    sensorData.print(altitude);
    sensorData.print("ft,PRES:");
    sensorData.println(pressure);
    sensorData.close();
  } else {
    Serial.println("Error opening sensordata.txt");
  }
}