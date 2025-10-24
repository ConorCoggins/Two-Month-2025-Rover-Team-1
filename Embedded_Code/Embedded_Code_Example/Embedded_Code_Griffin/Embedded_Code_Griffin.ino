/***************************************************************************
  This is a library for the BMP3XX temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP388 Breakout
  ----> http://www.adafruit.com/products/3966

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// #include <SD.h>
// #include <Stepper.h>

#define BMP_SCK 5
#define BMP_SDA 4
#define BNO_SCK 5
#define BNO_SDA 4


#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;


void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

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

    /* Initialise the sensor */
  
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
}

void loop() {
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
 delay(2000);

    /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  
  delay(100);
}