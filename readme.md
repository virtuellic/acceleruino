# acceleruino
Datalogger designed to track forces experienced during shipping

### Parts list:

+ Feather MO Adalogger
+ LIS3DH Accelerometer
+ Adafruit FONA 2G GSM Module
+ DS3231 external RTC module

### Requirements

1. attach loggers to floor of cargo containers
2. Measure a 3 axis accelerometer, each channel (axis) monitored at 500-1kHz sample rate.
3. measure only when triggered under each of the two different conditions below:
   + trigger when g-forces exceed a pre-set threshold, and log data for a specified duration or until G-forces subside, including pre-trigger data from buffer
   + log at set interval of 5 sec every 30 min
4. Want GPS data included in the logs (or cellular connection could provide location data, if possible)
5. battery powered and need 1-2 weeks worth of data

### TODO

+ [x] Convert to SDFat library
  + supports long filenames
  + better performance
+ [ ] Add constants for initialization parameters
  + range
  + sensitivity (ie. 2G - 16G)
  + sample rate
  + trigger threshold
  + wake interval (ie. 30 minutes)
  + wake log duration (ie. 5 sec)
+ [ ] Add config.ini to sd card for setting initialization parameters
+ [ ] setup trigger threshold interrupt
+ [ ] look into sleep / low power mode while waitig for trigger
+ [ ] find a 2G gsm provider in germany and buy prepaid sim card
+ [ ] add GPRS functionality
   + get time
   + get location
+ [ ] add external rtc interrupt handlers for wake interval sampling
