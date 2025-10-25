// Da Klankerz Code Jawn
// Watch yo self Jawn
// Arm in motion Jawn
// We doin it Jawn
// No sleep Jawn
// Slim 
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>
#include <Servo.h>

//Servos
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;


// SD Pinout Begin
const int _MISO = 16;
const int _MOSI = 19;
const int _CS = 17;
const int _SCK = 18;
// If you have all 4 DAT pins wired up to the Pico you can use SDIO mode
const int RP_CLK_GPIO = -1; // Set to CLK GPIO
const int RP_CMD_GPIO = -1; // Set to CMD GPIO
const int RP_DAT0_GPIO = -1; // Set to DAT0 GPIO. DAT1..3 must be consecutively connected.

File myFile;
//SD Pinout End

//BNO Code begin
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
//BNO Code ENd
//BMP Code Begin
#define BMP_SCK 5
#define BMP_SDA 4

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;
//BMP Code ENd



void setup(void)
{
  Serial.begin(115200);
  //Servo code begins
  servo1.attach(12);
  servo2.attach(13);
  servo3.attach(14);
  servo4.attach(15);
  //SErvo code ends

  // BNO Code Begin
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  //BNO Code ENd

  //BMP Code Begin
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  //BMP Code End

  //SD code Begin
  Serial.print("Initializing SD card...");

  bool sdInitialized = false;
  if (RP_CLK_GPIO >= 0) {
    // No special requirements on pin locations, this is PIO programmed
    sdInitialized = SD.begin(RP_CLK_GPIO, RP_CMD_GPIO, RP_DAT0_GPIO);
  } else {
    // Ensure the SPI pinout the SD card is connected to is configured properly
    // Select the correct SPI based on _MISO pin for the RP2040
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
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  //SD Code ENd


  delay(1000);
}


//SERVO CODE BEGIN
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
      if(sValue1 == "moveaxis1"){ // Code start 1700=90 degrees
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

       if(sValue1 == "moveaxis3"){ // Code start
        int secondCommaIndex = command.indexOf(',', firstCommaIndex + 1); //Dotn change
        String sValue2 = command.substring(firstCommaIndex + 1, secondCommaIndex);// DOnt change
        int value2 = sValue2.toInt(); //
        servo3.writeMicroseconds(value2); //change servo name
        Serial.print(value2); //dont change
      }

       if(sValue1 == "moveaxis4"){ // Code start
        int secondCommaIndex = command.indexOf(',', firstCommaIndex + 1); //Dotn change
        String sValue2 = command.substring(firstCommaIndex + 1, secondCommaIndex);// DOnt change
        int value2 = sValue2.toInt(); //
        servo4.writeMicroseconds(value2); //change servo name
        Serial.print(value2); //dont change
      }
    }
  }
}
// SERVO CODE END


void loop(void)
{
  readSerial();
  //BNO Code Begin
  //could add VECTOR_ACCELEROMETER, VECTOR_MAGNETOMETER,VECTOR_GRAVITY...
  sensors_event_t accelerometerData;
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  getAccelerometorData(&accelerometerData);
  //BNO Code End

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
  //BNO Code End
  
  //BMP Code Begin
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
  //BMP Code End


  //SD Code Begin
  myFile = SD.open("test.txt", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {
    // Serial.print("Writing to test.txt...");
    // myFile.println("testing 1, 2, 3.");
    Serial.print("Writing to test.txt...");
    // myFile.print(" Acceleration X=");
    // myFile.println(accelerometerData.x, DEC);
    // myFile.print(" Acceleration y=");
    // myFile.println(accelerometerData.y, DEC);
    //  myFile.print(" Acceleration z=");
    // myFile.println(accelerometerData.z, DEC);
    myFile.print(getAccelerometorData(&accelerometerData));
    myFile.print(", ");
    myFile.print(bmp.temperature);
    myFile.print(", ");
    myFile.print(bmp.pressure);
    myFile.print(", ");
    myFile.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    myFile.print(", ");
    myFile.print("Our Data Jawns!");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  // SD code end

}







String getAccelerometorData(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }
  String output = "\n";
  output.concat(event->acceleration.x);
  output.concat(", ");
  output.concat(event->acceleration.y);
  output.concat(", ");
  output.concat(event->acceleration.z);
  return output;
}


