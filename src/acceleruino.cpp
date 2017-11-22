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

int accelIntPin = 16;
int LIS3DH_ADDR = 0x18; // change this to 0x19 for alternative i2c address
int reading = 0; //counter for number of readings taken - for diagnostic use

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

unsigned int readAccelRegister(byte reg) {
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(LIS3DH_ADDR, 1);
  return Wire.read();
}


void writeAccelRegister(byte reg, byte data) {
  Wire.beginTransmission(LIS3DH_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
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

void init_ACC(void)
{
    // configurations for control registers
    writeAccelRegister(0x20, 0x57); //Write A7h into CTRL_REG1;      // Turn on the sensor, enable X, Y, Z axes with ODR = 100Hz normal mode.
    writeAccelRegister(0x21, 0x09); //Write 09h into CTRL_REG2;      // High-pass filter (HPF) enabled
    writeAccelRegister(0x22, 0x40); //Write 40h into CTRL_REG3;      // ACC AOI1 interrupt signal is routed to INT1 pin.
    writeAccelRegister(0x23, 0x00); //Write 00h into CTRL_REG4;      // Full Scale = +/-2 g
    writeAccelRegister(0x24, 0x08); //Write 08h into CTRL_REG5;      // Default value is 00 for no latching. Interrupt signals on INT1 pin is not latched.
                                                                //Users don’t need to read the INT1_SRC register to clear the interrupt signal.
    // configurations for wakeup and motionless detection
    writeAccelRegister(0x32, 0x10); //Write 10h into INT1_THS;          // Threshold (THS) = 16LSBs * 15.625mg/LSB = 250mg.
    writeAccelRegister(0x33, 0x00); //Write 00h into INT1_DURATION;     // Duration = 1LSBs * (1/10Hz) = 0.1s.
    //readRegister();  //Dummy read to force the HP filter to set reference acceleration/tilt value
    writeAccelRegister(0x30, 0x2A); //Write 2Ah into INT1_CFG;          // Enable XLIE, YLIE, ZLIE interrupt generation, OR logic.

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
  while (!accel.begin(LIS3DH_ADDR)) {
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
  init_ACC();

  Serial.print("Range = "); Serial.print(2 << accel.getRange());
  //Serial.print(accel.getRange(), BIN);
  Serial.println("G");

  Serial.println("Success initializing accelerometer");
  // END Accelerometer initialization

  delay(6000);

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

  if(digitalRead(accelIntPin) == HIGH) {
    Serial.print("  \tinterrupt: ");
    Serial.print(reading++); Serial.print(",  ");
    Serial.print(readAccelRegister(0x21)); Serial.print(",  "); //read register to reset high-pass filter
    Serial.print(readAccelRegister(0x26)); Serial.print(",  "); //read register to set reference acceleration
    Serial.print(readAccelRegister(LIS3DH_REG_INT1SRC)); Serial.print(",  "); //Read INT1_SRC to de-latch;
  }
  Serial.println();

  logAccelData(accel.x, accel.y, accel.z);


  delay(1);


}
