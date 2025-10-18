// Example Code
// Include Libraries
#include <SD.h>

//Define Chip Select(CS) Pin, GPIO
const int chipSelect = 17; //GPIO 17

//Example COde
const String dataValues[] = {
  "<D: 12.5, 23.1, 34.2>",
  "<D: 13.6, 22.5, 31.9>",
  "<D: 11.4, 25.6, 30.0>",
  "<D: 14.1, 24.3, 29.8>",
  "<D: 10.9, 22.0, 32.5>"
};

int randomIndex; // A random index to retrieve data from array

File dataLog; // Declare the log file to be used in the loop


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //Initiate Serial Monitor
  while(!Serial); //Wait for serial monitor to initialize

  while (!SD.begin(chipSelect)) {
    Serial.println("Failed to initialize SD Card.");
  }
  Serial.println("SD card itialized successfully.");
}

void loop() {
  // put your main code here, to run repeatedly:
  //Open a file in the SD Card
  // Only 1 file can be opened at a time
  // Specifiying only the name or having a single forward
  // slash means the file will be located in the root directory
  File dataLog = SD.open("dataLog.txt", FILE_WRITE);

  //Check if file is open
  //IF the file is OK, Write to it
  if (dataLog) {
    randomIndex = random(0,5); // A random index between 0 and 4
    dataLog.println(dataValues[randomIndex]); //Write a random data line to the file
    dataLog.close(); //Close the file after we're done with it
    Serial.println(dataValues[randomIndex]); //Display the same line in  the serial monitor to compare integrity
  } else{
    Serial.println("Failed to open dataLog.txt."); //Display error message in the serial monitor
  }

  delay(1000); //Wait for a second between loggings

  
}
