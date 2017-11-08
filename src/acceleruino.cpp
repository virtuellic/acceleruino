// Basic demo for accelerometer readings from Adafruit LIS3DH

#include <Wire.h>
#include "Adafruit_LIS3DH.h"
#include <Adafruit_Sensor.h>

#include <SdFat.h>
#include "RTClib.h"

const int chipSelect = 4;

String dataString = "";
File dataFile;

SdFat SD;

// I2C
Adafruit_LIS3DH lis = Adafruit_LIS3DH();
RTC_Millis rtc;

void dateTime(uint16_t* date, uint16_t* time) {

  DateTime now = rtc.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

void setup(void) {

  while (!Serial);
  Serial.begin(9600);
  Serial.println("LIS3DH test!");

  while (!lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    delay (1000);
  }
  Serial.println("LIS3DH found!");

  lis.setRange(LIS3DH_RANGE_2_G);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << lis.getRange());
  Serial.println("G");

  lis.setDataRate(LIS3DH_DATARATE_25_HZ);

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect, SD_SCK_MHZ(50))) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  Serial.println("Opening dataFile.");

  rtc.begin(DateTime(F(__DATE__), F(__TIME__)));
  DateTime now = rtc.now();

  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();


  char fileName[26];
  snprintf(fileName, 26, "data_%i-%i-%i_%i%i%i.txt", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());



  Serial.println(fileName);

  SdFile::dateTimeCallback(dateTime);

  dataFile = SD.open(fileName, O_CREAT | O_WRITE | O_EXCL);
  if(dataFile) {
    Serial.println("Success opening file");
  }
  else {
    Serial.println("Error opening file");
    return;
  }
}

void loop() {
  lis.read();      // get X Y and Z data at once
  // Then print out the raw data
  Serial.print("X:  "); Serial.print(lis.x);
  Serial.print("  \t\t\tY:  "); Serial.print(lis.y);
  Serial.print("  \tZ:  "); Serial.print(lis.z);

  /* Or....get a new sensor event, normalized */
  sensors_event_t event;
  lis.getEvent(&event);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
  Serial.print(" \tY: "); Serial.print(event.acceleration.y);
  Serial.print(" \tZ: "); Serial.print(event.acceleration.z);
  Serial.println(" m/s^2 ");

  dataString = "";
  dataString += String(lis.x);
  dataString += ",";
  dataString += String(lis.y);
  dataString += ",";
  dataString += String(lis.z);

  if (dataFile) {
    dataFile.println(dataString);
    dataFile.flush();
    // print to the serial port too:
    Serial.println(dataString);
  }
  else {
    Serial.println("File not open");
    return;
  }
  delay(100);


}
