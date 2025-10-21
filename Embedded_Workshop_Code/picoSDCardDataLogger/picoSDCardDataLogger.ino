// this is copied from the website Jack sent me. There are prerequisites when it comes 
// to certain wirings for the SD card. You can find it in the Software-help chat

#include <SD.h>

const int chipSelect = 17; // GP pin ID on Pico for chip select

// Sample data logging values
const String dataValues[] = {
  "<D: 12.5, 23.1, 34.2>",
  "<D: 13.6, 22.5, 31.9>",
  "<D: 11.4, 25.6, 30.0>",
  "<D: 14.1, 24.3, 29.8>",
  "<D: 10.9, 22.0, 32.5>"
};

int randomIndex; // A random index to retrieve data from the array

File dataLog; // Declare the log file to be used in the loop

//there is another alternative for the following begin() function if this one is incorrect

bool SDClass::begin(uint32_t clock, uint8_t csPin)

//"We can observe that there exists a value for SPI_HALF_SPEED. Let’s find it.""
//"In utility/SD2Card.h is the value for our default clock setting."

/** Set SCK rate to F_CPU/4. See Sd2Card::setSckRate(). */

uint8_t const SPI_HALF_SPEED = 1;

case 1: settings = SPISettings(4000000, MSBFIRST, SPI_MODE0); break;

void setup() {}
  Serial.begin(115200); // Initiate the serial monitor
  while(!Serial); // Wait for the serial monitor to initialize

  while (!SD.begin(chipSelect)) {
    Serial.println("Failed to initialize SD card.");
  }
  Serial.println("SD card initialized successfully.");
}

void loop() {
    // Open a file in the SD card
  // Only one file can be open at a time
  // Specifying only the name or having a single forward
  // slash means the file will be located in the root directory
  File dataLog = SD.open("dataLog.txt", FILE_WRITE);

}

//"The file object overloads the boolean operator [16]. 
//It’s the same as file.isOpen(). We can use the object directly.""
File::operator bool() {

if (_file) {

return _file->isOpen();

}

return false;

}

// If the file is OK, write to it
  if (dataLog) {
    randomIndex = random(0, 5); // A random index between 0 and 4
    dataLog.println(dataValues[randomIndex]); // Write a random data line to the file
    dataLog.close(); // Close the file after we're done with it
    Serial.println(dataValues[randomIndex]); // Display the same line in the serial monitor to compare the integrity
  } else {
    Serial.println("Failed to open dataLog.txt."); // Display error message in the serial monitor
  }

  delay(1000); // Wait for a second between loggings
}