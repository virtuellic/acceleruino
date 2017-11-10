#include <Wire.h>
#include "Adafruit_LIS3DH.h"
#include <Adafruit_Sensor.h>

#include <SdFat.h>
#include "RTClib.h"

#define RANGE LIS3DH_RANGE_2_G  // LIS3DH_RANGE_<N>_G where N in (2, 4, 8, 16)
/* Valid Rate Values
LIS3DH_DATARATE_400_HZ     = 0b0111, //  400Hz
LIS3DH_DATARATE_200_HZ     = 0b0110, //  200Hz
LIS3DH_DATARATE_100_HZ     = 0b0101, //  100Hz
LIS3DH_DATARATE_50_HZ      = 0b0100, //   50Hz
LIS3DH_DATARATE_25_HZ      = 0b0011, //   25Hz
LIS3DH_DATARATE_10_HZ      = 0b0010, // 10 Hz
LIS3DH_DATARATE_1_HZ       = 0b0001, // 1 Hz
LIS3DH_DATARATE_POWERDOWN  = 0,
LIS3DH_DATARATE_LOWPOWER_1K6HZ  = 0b1000,
LIS3DH_DATARATE_LOWPOWER_5KHZ  =  0b1001
*/
#define RATE LIS3DH_DATARATE_25_HZ

const int chipSelect = 4;
volatile int stopFlag = 0;

String dataString = "";
File dataFile;

SdFat SD;

// I2C
Adafruit_LIS3DH accel = Adafruit_LIS3DH();

RTC_DS3231 rtc;

void sd_dateTime_callback(uint16_t* date, uint16_t* time) {

  DateTime now = rtc.now();

  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(now.year(), now.month(), now.day());

  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

DateTime getGPRSDateTime() {

  return DateTime(F(__DATE__), F(__TIME__));

}
void logAccelData(int x, int y, int z) {

  DateTime logtime = rtc.now();
  char ts[18];
  snprintf(ts, sizeof(ts), "%i-%i-%i %i%i%i", logtime.year(), logtime.month(), logtime.day(), logtime.hour(), logtime.minute(), logtime.second());

  Serial.print(ts);
  Serial.print("  \tX:  "); Serial.print(x);
  Serial.print("  \tY:  "); Serial.print(y);
  Serial.print("  \tZ:  "); Serial.println(z);

  if (dataFile) {
    dataFile.print(ts);
    dataFile.print(",");
    dataFile.print(x);
    dataFile.print(",");
    dataFile.print(y);
    dataFile.print(",");
    dataFile.println(z);

    dataFile.flush();

  }
  else {
    Serial.println("File not open");
    stopFlag = 1;
    return;
  }


}

void setup(void) {

  while (!Serial);
  Serial.begin(9600);
  Serial.println();
  Serial.println("##############################");
  Serial.println("Acceleruino Data Logger");
  Serial.println("Measurement Solutions LLC");
  Serial.println("www.MEASOL.com");
  Serial.println("##############################");
  Serial.println();

  // BEGIN Accelerometer Initialization
  Serial.println("Initializing accelerometer....");
  int RETRY_VAL = 10;
  int cnt = 0;
  while (!accel.begin(0x18)) {
    Serial.print("Error inializing accelerometer...");
    Serial.println("retrying in 1 second");
    delay (1000);
    cnt++;
    if(cnt > RETRY_VAL) {
      stopFlag = 1;
      Serial.println("Accelerometer failed to initialize");
      return;
    }
  }

  accel.setRange(RANGE);   // 2, 4, 8 or 16 G!

  Serial.print("Range = "); Serial.print(2 << accel.getRange());
  Serial.println("G");

  accel.setDataRate(RATE);

  Serial.println("Success initializing accelerometer");
  // END Accelerometer initialization

  // BEGIN RTC initialization
  Serial.println("Initializing DS3231 RTC");
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    stopFlag = 1;
    return;
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    rtc.adjust(getGPRSDateTime());
  }
  Serial.println("Success initializing RTC");
  // END RTC initialization

  // BEGIN SD Card initialization
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect, SD_SCK_MHZ(50))) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    stopFlag = 1;
    return;
  }
  Serial.println("card initialized.");
  // END SD Card initialization

  // BEGIN creating new logfile
  DateTime now = rtc.now();

  char fileName[26];
  snprintf(fileName, 26, "data_%i-%i-%i_%i%i%i.txt", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  Serial.print("Creating logfile: ");
  Serial.println(fileName);

  SdFile::dateTimeCallback(sd_dateTime_callback); // sets callback for file timestamp

  dataFile = SD.open(fileName, O_CREAT | O_WRITE | O_EXCL);
  if(dataFile) {
    Serial.println("Success opening file");
  }
  else {
    Serial.println("Error opening file");
    stopFlag = 1;
    return;
  }
  // END logfile creation

}

void loop() {

  //stopFlag = 1;
  while(stopFlag);

  accel.read();      // get X Y and Z data at once

  logAccelData(accel.x, accel.y, accel.z);


  delay(1);


}
