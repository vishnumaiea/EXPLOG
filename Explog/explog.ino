
//====================================================================//
//  EXPLOG : Exploration Logger                                       //
//                                                                    //
// This is a battery powered, handheld device for travelers and       //
// explorers. It will monitor and log your GPS location, direction,   //
// speed and acceleration while you travel and also the temperature,  //
// pressure, altitude, humidity, noise level and orientation of the   //
// place where you are. So it's an all-in-one device than can gather  //
// all the data of the places you'll explore.                         //
//                                                                    //
//  Author : Vishnu M Aiea                                            //
//  Website : www.vishnumaiea.in                                      //
//  Date created : 2:53 PM 27-09-2017, Wednesday                      //
//  Last modified : 9:02 PM 13-03-2018, Tuesday                       //
//  File version : 1.0                                                //
//                                                                    //
//  This was ported from ESp8266-NodeMCU                              //
//====================================================================//

// Changelog:
//12:21:20 PM, 01-10-2017, Sunday
//Fixed the Exception 3 fatal error by reoving PROGMEM function
//07:54:09 PM, 01-10-2017, Sunday
//Keeping separate files for Nano and NodeMCU
//11:29:02 AM, 03-10-2017, Tuesday
//GPS found to be working on software serial pins
//09:40:44 PM, 03-10-2017, Tuesday
//For some reason, the 38400 baud rate doesn't work
//09:41:17 PM, 03-10-2017, Tuesday
//added a line to disable the SW WDT that was hindering with the gps
//reading function
//11:18:43 PM, 03-10-2017, Tuesday
//Added formatted time support
//12:58:00 AM, 04-10-2017, Wednesday
//Fixed display flickering

//===================================================================//

#include <SPI.h>
#include <Adafruit_GFX.h>
#include "Adafruit_PCD8544.h"
#include <SoftwareSerial.h>
#include <ifx_dps310_esp.h>
#include <SlowSoftI2CMaster.h>
#include <SlowSoftWire.h>
#include <SD.h>
#include <TimeLib.h>

//===================================================================//

#include <SFE_BMP180.h>
#include <Wire.h>

SFE_BMP180 bmp180sensor;

double bmpPressure;
double bmpTemperature;

//===================================================================//

#define INVALID -1

const int analogPin = A0;     // the number of the pushbutton pin

const int BUTTON_UP = 1;
const int BUTTON_DOWN = 2;
const int BUTTON_RIGHT = 3;
const int BUTTON_LEFT = 4;
const int BUTTON_SAVE = 5;

const int BUTTON_UP_LOW = 970;
const int BUTTON_UP_HIGH = 1024;
const int BUTTON_DOWN_LOW = 850;
const int BUTTON_DOWN_HIGH = 950;
const int BUTTON_RIGHT_LOW = 700;
const int BUTTON_RIGHT_HIGH = 800;
const int BUTTON_LEFT_LOW = 400;
const int BUTTON_LEFT_HIGH = 650;
const int BUTTON_SAVE_LOW = 250;
const int BUTTON_SAVE_HIGH = 350;

int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

//===================================================================//
//declarations and definitions for DPS310 sensor

// uint8_t temp_mr = 4; //temp measure rate 0-7
// uint8_t temp_osr = 4; //temp oversampling rate 0-7
// uint8_t prs_mr = 4; //pressure measure rate 0-7
// uint8_t prs_osr = 4; //pressure oversampling rate 0-7

uint8_t pressureCount = 3;
int32_t pressure[3]; //in Pascals
uint8_t temperatureCount = 3;
int32_t temperature[3]; //in degrees celsius

double temperatureValue; //degrees celsius
double pressureValue; //Pascals
double baselinePressure; //in Pascals
double baselineAltitude; //in metres

boolean sensorInitialized = false;

#define SEA_LEVEL_PRESSURE 101325 //in Pascals

// SlowSoftWire Wire = SlowSoftWire(4, 5); //GPIO 4 to SDA and GPIO 5 to SCL

//===================================================================//
//declarations and definitions for LCD

#define CLEAR_DISPLAY LCD_5110.clearDisplay()
#define UPDATE_DISPLAY LCD_5110.display()

#define DISPLAY_WIDTH 84
#define DISPLAY_HEIGHT 48
#define MONO_WIDTH 5
#define MONO_HEIGHT 7

#define ALIGN_LEFT 1
#define ALIGN_CENTER 2
#define ALIGN_RIGHT 3

//===================================================================//
//declarations for GPS module

#define GPS_RX 0 //D3
#define GPS_TX 16 //D0

SoftwareSerial gpsSerial = SoftwareSerial(GPS_RX, GPS_TX); //Rx and Tx

const String string_acquiring = "ACQUIRING";
const String string_colon = ":";
const String string_hyphen = "-";
const String string_utc = " UTC";
const String string_latitude = "LA:";
const String string_longitude = "LO:";
const String string_speed = "S:";
const String string_speedUnit = " km/h";
const String string_trackingOn [] = "TRACKING ON";
const String string_trackingOff [] = "TRACKING OFF";
const String string_toStart [] = "TO START";
const String string_pressSave [] = "PRESS SAVE";
const String string_temperature = "T:";
const String string_pressure = "P:";
const String string_pressureUnit = " kPa";
const String string_altitude = "AL:";
const String string_altitudeUnit = " m";
const String string_space = " ";
const String string_star = " *";
const String string_celsius = " C";
const String string_gpmrc [] = "$GPMRC";

char latitudeBuffer[10];   // latitude array - length = 9
char longitudeBuffer[11];  // longitude array - length = 10
char speedBuffer[4];  // GPS speed array
char latitudeDirBuffer; // latitude direction
char longitudeDirBuffer; // longitude direction
char timeBuffer[7]; //UTC time
char dateBuffer[7]; //UTC date
char satelliteCountBuffer[5]; //satellites in view

bool gpsFixed = false;

//===================================================================//
//display data declarations

#define PAGE_DEFAULT 0
#define PAGE_LOCATION 1
#define PAGE_ALTITUDE 2
#define PAGE_TIME 3

String temperatureString;
String pressureString;
String altitudeString;
String absAltitudeString;
String relAltitudeString;
String latitudeString;
String longitudeString;
String speedString;
String timeString;
String dateString;
String dayString;
String timeZoneString;
String savedEntriesCountString;
String trackingEntriesCountString;
String trackingRateString;
String satelliteString;

// int currentPageId = PAGE_DEFAULT;
// int currentPageId = PAGE_TIME;
int currentPageId = PAGE_ALTITUDE;
int lastPageId = 0;
// int currentPageId = PAGE_LOCATION;

//===================================================================//
//status variables

bool trackingOn = true;;
bool relAltitudeMode = false;
uint32_t awake_time = 0; //awake time of Arduino - reset to 0 after 50 days (2^32 ms)

//===================================================================//
//decalations for Nokia 5110 LCD

//pins for software SPI - uncomment this when using SW SPI
#define LCD_RST 15 //D8
#define LCD_CS 13 //D7
#define LCD_DC 12 //D6
#define LCD_DIN 14 //D5
#define LCD_SCK 2 //D4

//pins for hardware SPI - uncomment this when using HW SPI
// #define LCD_RST 2 //D4
// #define LCD_CS 0 //D3 - separate CS pin so that multiple devices can share SPI
// #define LCD_DC 16 //D0 - this pin can even be the MISO pin when other deviced are not selected
// #define LCD_DIN 13 //D7
// #define LCD_SCK 14 //D5

//select any of the following
Adafruit_PCD8544 LCD_5110 = Adafruit_PCD8544 (LCD_SCK, LCD_DIN, LCD_DC, LCD_CS, LCD_RST); //software SPI
// Adafruit_PCD8544 LCD_5110 = Adafruit_PCD8544(LCD_DC, LCD_CS, LCD_RST); //hardware SPI


//===================================================================//
//setup function runs once

void setup()   {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  Serial.println("--- PORTABLE GPS LOGGER ---");
  Serial.println("by Vishnu M Aiea");
  Serial.println();

  gpsSerial.begin(9600);
  initializeGPS(); //initialize GPS

  LCD_5110.begin(); //initialize LCD
  LCD_5110.setContrast(53);
  CLEAR_DISPLAY;

  // initializeDPS310();
  initializeBMP180();

  showWelcomeScreen(); //shows splash screen
  delay(2000);

  setTime(1507025298); //3:38 PM,, 03-10-2017, Tue

  CLEAR_DISPLAY;
}

//===================================================================//
//infinite loop

void loop() {
  ESP.wdtDisable(); //disable SW WDT - important! This will avoid a lot of mess

  switch (currentPageId) {
    case PAGE_DEFAULT:
      showDefaultInfo();
      lastPageId = PAGE_DEFAULT;
      // delay(1);
      break;
    case PAGE_LOCATION:
      showLocationInfo();
      lastPageId = PAGE_LOCATION;
      // delay(1);
      break;
    case PAGE_ALTITUDE:
      showAltitudeInfo();
      lastPageId = PAGE_ALTITUDE;
      delay(500);
      break;
    case PAGE_TIME:
      showTimeInfo();
      lastPageId = PAGE_TIME;
      // delay(1);
      break;
    default:
      currentPageId = lastPageId;
  }

  int currentButton = readKeypad();
  Serial.print("currentButton = ");
  Serial.println(currentButton);
  Serial.println();

  if((currentButton >= 0) && (currentButton <= 5)) {
    if(currentButton == BUTTON_UP) currentPageId = PAGE_LOCATION;
    if(currentButton == BUTTON_DOWN) currentPageId = PAGE_ALTITUDE;
    if(currentButton == BUTTON_RIGHT) currentPageId = PAGE_TIME;
    if(currentButton == BUTTON_LEFT) currentPageId = PAGE_DEFAULT;
  }
  
  // if(gpsSerial.available())
  // Serial.print(gpsSerial.readStringUntil('*'));

  awake_time = millis(); //calculate the time span MCU is ON
}

//===================================================================//

int readKeypad () {
  // read the state of the switch into a local variable:
  int reading = analogRead(analogPin);
  Serial.print("ADC Reading = ");
  Serial.println(reading);
  Serial.println();

  int tmpButtonState = LOW;             // the current reading from the input pin

  if(reading>BUTTON_SAVE_LOW && reading<BUTTON_SAVE_HIGH){
    //Read switch 5
    tmpButtonState = BUTTON_SAVE;
  }
  else if(reading>BUTTON_LEFT_LOW && reading<BUTTON_LEFT_HIGH){
    //Read switch 4
    tmpButtonState = BUTTON_LEFT;
  }
  else if(reading>BUTTON_RIGHT_LOW && reading<BUTTON_RIGHT_HIGH){
    //Read switch 3
    tmpButtonState = BUTTON_RIGHT;
  }
  else if(reading>BUTTON_DOWN_LOW && reading<BUTTON_DOWN_HIGH){
    //Read switch 2
    tmpButtonState = BUTTON_DOWN;
  }
  else if(reading>BUTTON_UP_LOW && reading<BUTTON_UP_HIGH){
    //Read switch 1
    tmpButtonState = BUTTON_UP;
  }
  else{
    //No button is pressed;
    tmpButtonState = currentPageId;
  }

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to a buttonState),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (tmpButtonState != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    buttonState = tmpButtonState;
    Serial.println(buttonState);
  }

  // save the reading.  Next time through the loop,
  // it'll be the lastButtonState:
  lastButtonState = tmpButtonState;

  // set the LED using the state of the button for testing:
  switch(buttonState){
    case BUTTON_UP:
      return BUTTON_UP;
      break;
    case BUTTON_DOWN:
      return BUTTON_DOWN;
      break;
    case BUTTON_RIGHT:
      return BUTTON_RIGHT;
      break;
    case BUTTON_LEFT:
      return BUTTON_LEFT;
      break;
    case BUTTON_SAVE:
      return BUTTON_SAVE;
      break;
  }
  return INVALID;
}

//===================================================================//

void showWelcomeScreen () {
  CLEAR_DISPLAY;
  LCD_5110.setTextSize(1);
  LCD_5110.setTextColor(BLACK);
  updateAlignedText("PORTABLE", 3, ALIGN_CENTER); //string, y, alignment
  updateAlignedText("GPS LOGGER", 14, ALIGN_CENTER); //string, y, alignment
  updateAlignedText("- - - -", 24, ALIGN_CENTER); //string, y, alignment
  updateAlignedText("VISHNU M AIEA", 37, ALIGN_CENTER); //string, y, alignment
  UPDATE_DISPLAY;
}

//===================================================================//

void showDefaultInfo () {
  getTimeInfo(); //fetch time + location from GPS
  getLocationInfo();
  getAltitude(); //fetch altitude
  // CLEAR_DISPLAY;
  LCD_5110.setTextSize(1);
  LCD_5110.setTextColor(BLACK);
  updateAlignedText(timeString, 2, ALIGN_CENTER); //time string
  updateAlignedText(longitudeString, 15, ALIGN_CENTER); //longitude string
  updateAlignedText(latitudeString, 25, ALIGN_CENTER); //latitude string
  updateAlignedText(altitudeString, 38, ALIGN_CENTER); //abs altitude string
  UPDATE_DISPLAY;
}

//===================================================================//

void showTimeInfo () {
  getTimeInfo(); //uptade the time and date strings
  // CLEAR_DISPLAY;
  LCD_5110.setTextSize(1);
  LCD_5110.setTextColor(BLACK);
  updateAlignedText(timeString, 2, ALIGN_CENTER); //string, y, alignment - time string
  updateAlignedText(timeZoneString, 14, ALIGN_CENTER); //string, y, alignment
  updateAlignedText(dateString, 26, ALIGN_CENTER); //string, y, alignment - date string
  updateAlignedText(dayString, 38, ALIGN_CENTER); //string, y, alignment
  UPDATE_DISPLAY;
}

//===================================================================//

void getTimeInfo() {
  // receiveGPRMC();
  if(strlen(timeBuffer) > 6) timeBuffer[5] = 0; //we need only 6 values
  if(strlen(dateBuffer) > 6) dateBuffer[5] = 0;

  if((strlen(timeBuffer) == 6) && (strlen(dateBuffer) == 6)) {
    //first set the current time from GPS
    setTime((String(timeBuffer).substring(0, 2)).toInt(), //UTC hours
           (String(timeBuffer).substring(2, 4)).toInt(), //UTC minutes
           (String(timeBuffer).substring(4)).toInt(), //UTC seconds
           (String(dateBuffer).substring(0, 2)).toInt(), //UTC day
           (String(dateBuffer).substring(2, 4)).toInt(), //UTC month
           String("20" + (String(dateBuffer).substring(4))).toInt()); //UTC year
    //then adjust the time with offset
    adjustTime(19800); //GMT +5:30
  }

  if(String(hourFormat12()).length() == 1) timeString = "0" + String(hourFormat12()) + ":";
  else timeString = String(hourFormat12()) + ":";

  if(String(minute()).length() == 1) timeString += "0" + String(minute()) + ":";
  else timeString += String(minute()) + ":";

  if(String(second()).length() == 1) timeString += "0" + String(second());
  else timeString += String(second());

  if(isAM()) timeString += " AM";
  else timeString += " PM";

  if(String(day()).length() == 1) dateString = "0" + String(day()) + "-";
  else dateString = String(day()) + "-";

  if(String(month()).length() == 1) dateString += "0" + String(month()) + "-";
  else dateString += String(month()) + "-";

  dateString += String(year());

  dateString = String(day()) + "-" + String(month()) + "-" + String(year());

  switch(weekday()) {
    case 1:
      dayString = "SUNDAY";
      break;
    case 2:
      dayString = "MONDAY";
      break;
    case 3:
      dayString = "TUESDAY";
      break;
    case 4:
      dayString = "WEDNESDAY";
      break;
    case 5:
      dayString = "THURSDAY";
      break;
    case 6:
      dayString = "FRIDAY";
      break;
    case 7:
     dayString = "SATURDAY";
  }
  // timeString = (strlen(timeBuffer) == 6) ? String(String(timeBuffer).substring(0, 2) + string_colon + String(timeBuffer).substring(2, 4) + string_colon + String(timeBuffer).substring(4) + string_utc) : string_acquiring;
  // dateString = (strlen(dateBuffer) == 6) ? String(String(dateBuffer).substring(0, 2) + string_hyphen + String(dateBuffer).substring(2, 4) + string_hyphen + String(dateBuffer).substring(4)) : string_acquiring;
  timeZoneString = "GMT +5:30 IST";
  UPDATE_DISPLAY;
}

//===================================================================//

void updateTime() {
  if(currentPageId == PAGE_DEFAULT) {
    getTimeInfo();
    updateAlignedText(timeString, 2, ALIGN_CENTER); //time string
    updateAlignedText(altitudeString, 38, ALIGN_CENTER); //abs altitude string
    if(!gpsFixed) {
      updateAlignedText("ACQUIRING", 15, ALIGN_CENTER); //longitude string
      updateAlignedText("ACQUIRING", 25, ALIGN_CENTER); //latitude string
    }
    UPDATE_DISPLAY;
  }
}
//===================================================================//

void showDataSavedInfo () {
  CLEAR_DISPLAY;
  LCD_5110.setTextSize(1);
  LCD_5110.setTextColor(BLACK);
  updateAlignedText("DATA SAVED", 10, ALIGN_CENTER); //string, y, alignment
  updateAlignedText(savedEntriesCountString, 26, ALIGN_CENTER); //string, y, alignment
  UPDATE_DISPLAY;
}

//===================================================================//

void showAltitudeInfo () {
  getAltitude();
  CLEAR_DISPLAY;
  LCD_5110.setTextSize(1);
  LCD_5110.setTextColor(BLACK);
  // updateAlignedText(absAltitudeString, 2, ALIGN_CENTER); //abs altitude string
  // updateAlignedText(relAltitudeString, 14, ALIGN_CENTER); //rel altitude string
  // updateAlignedText(pressureString, 26, ALIGN_CENTER); //pressure string
  // updateAlignedText(temperatureString, 38, ALIGN_CENTER); //temperature string

  updateText(absAltitudeString, 0, 2); //abs altitude string
  updateText(relAltitudeString, 0, 14); //rel altitude string
  updateText(pressureString, 0, 26); //pressure string
  updateText(temperatureString, 0, 38); //temperature string
  LCD_5110.setCursor(42,36);
  LCD_5110.write(9);
  UPDATE_DISPLAY;
}

//===================================================================//

void showLocationInfo () {
  getLocationInfo();
  CLEAR_DISPLAY;
  LCD_5110.setTextSize(1);
  LCD_5110.setTextColor(BLACK);
  updateAlignedText(longitudeString, 2, ALIGN_CENTER); //longitude string
  updateAlignedText(latitudeString, 14, ALIGN_CENTER); //latitude string
  updateAlignedText(speedString, 26, ALIGN_CENTER); //speed string
  updateAlignedText(satelliteString, 38, ALIGN_CENTER); //satellite string
  UPDATE_DISPLAY;
}

//===================================================================//

void getLocationInfo () {
  receiveGPRMC();
  receiveGPGSV();
  longitudeString = (strlen(longitudeBuffer) == 10) ? String(string_longitude + String(longitudeBuffer) + string_space + String(longitudeDirBuffer)) : string_acquiring;
  latitudeString = (strlen(latitudeBuffer) == 9) ? String(string_latitude + String(latitudeBuffer) + string_space + String(latitudeDirBuffer)) : string_acquiring;
  speedString = (strlen(speedBuffer) == 3) ? String(string_speed + String((String(speedBuffer).toFloat()) * 1.85) + string_speedUnit) : string_acquiring;
  satelliteString = "SAT: " + String(satelliteCountBuffer);
}

//===================================================================//
//not used wihout SD card feature

// void showTrackingOnInfo () {
//   CLEAR_DISPLAY;
//   LCD_5110.setTextSize(1);
//   LCD_5110.setTextColor(BLACK);
//   updateAlignedText("TRACKING ON", 4, ALIGN_CENTER); //string, y, alignment
//   updateAlignedText(lcdString, 19, ALIGN_CENTER); //string, y, alignment
//   updateAlignedText(lcdString, 34, ALIGN_CENTER); //string, y, alignment
// }

//===================================================================//
//not used wihout SD card rfeature

// void showTrackingOffInfo () {
//   CLEAR_DISPLAY;
//   LCD_5110.setTextSize(1);
//   LCD_5110.setTextColor(BLACK);
//   updateAlignedText("TRACKING OFF", 4, ALIGN_CENTER); //string, y, alignment
//   updateAlignedText("PRESS SAVE", 21, ALIGN_CENTER); //string, y, alignment
//   updateAlignedText("TO START", 32, ALIGN_CENTER); //string, y, alignment
// }

//===================================================================//
//fetches the RMC header from the outputs

void receiveGPRMC() {
  int32_t spanTime = 0;
  byte byteBuffer = 0; // receive serial data byte
  uint8_t finishStatus = 0;  // indicates end of message
  uint8_t positionCount = 0;  // position counter
  uint8_t latitudeBufferCount = 0;  // latitude data counter
  uint8_t longitudeBufferCount = 0;  // longitude data counter
  uint8_t speedBufferCount = 0;  //speed data counter
  uint8_t readFlag = 0;  // GPS flag
  uint8_t commaCount = 0;  // comma counter
  uint8_t timeBufferCount = 0;
  uint8_t dateBufferCount = 0;

  Serial.println("Fetching GPRMC data..");
  Serial.println();

  awake_time = millis();
  // ESP.wdtDisable();

  while ((finishStatus == 0) && (spanTime < 100)) { //will take ~1 s to acquire
    updateTime(); //keep the time ticking while fetching GPS data
    while ((gpsSerial.available() > 0) && (finishStatus == 0)) { // Check GPS data
      byteBuffer = gpsSerial.read();
      readFlag = 1;

      if (byteBuffer == '$' && positionCount == 0) positionCount = 1;
      if (byteBuffer == 'G' && positionCount == 1) positionCount = 2;
      if (byteBuffer == 'P' && positionCount == 2) positionCount = 3;
      if (byteBuffer == 'R' && positionCount == 3) positionCount = 4;
      if (byteBuffer == 'M' && positionCount == 4) positionCount = 5;
      if (byteBuffer == 'C' && positionCount == 5) positionCount = 6;

      if (positionCount == 6 && byteBuffer == ',') { // count commas in message
        commaCount++;
        readFlag = 0;
      }

      if (commaCount == 1 && readFlag == 1) {
        timeBuffer[timeBufferCount++] =  byteBuffer; // Latitude
        readFlag = 0;
      }

      if (commaCount == 3 && readFlag == 1) {
        latitudeBuffer[latitudeBufferCount++] =  byteBuffer; // Latitude
        readFlag = 0;
      }

      if (commaCount == 4 && readFlag == 1) {
        latitudeDirBuffer =  byteBuffer; // Latitude Direction N/S
        readFlag = 0;
      }

      if (commaCount == 5 && readFlag == 1) {
        longitudeBuffer[longitudeBufferCount++] =  byteBuffer; // Longitude
        readFlag = 0;
      }

      if (commaCount == 6 && readFlag == 1) {
        longitudeDirBuffer =  byteBuffer; // Longitude Direction E/W
        readFlag = 0;
      }

      if (commaCount == 7 && readFlag == 1) {
        speedBuffer[speedBufferCount++] =  byteBuffer; // Speed in knot
        readFlag = 0;
      }

      if (commaCount == 8 && readFlag == 1) {
        dateBuffer[dateBufferCount++] =  byteBuffer; // Speed in knot
        readFlag = 0;
      }

      if (byteBuffer == '*' && commaCount >= 7) { // end of GPRMC message
        longitudeBuffer[longitudeBufferCount] = 0;
        latitudeBuffer[latitudeBufferCount] = 0;
        speedBuffer[speedBufferCount] = 0;
        timeBuffer[timeBufferCount] = 0;
        dateBuffer[dateBufferCount] = 0;
        commaCount = 0;
        latitudeBufferCount = 0;
        longitudeBufferCount = 0;
        speedBufferCount = 0;
        timeBufferCount = 0;
        dateBufferCount = 0;
        readFlag = 0;
        finishStatus = 1;
        if((strlen(latitudeBuffer) > 5) && (strlen(longitudeBuffer) > 5)) {
          gpsFixed = true;
          Serial.println("GPRMC aquisition success");
          Serial.println();
          Serial.print("GPRMC Acquisition Time: ");
          Serial.println(spanTime);
        }
      }
    }
    spanTime = (int32_t) millis() - awake_time;
  }
  if(finishStatus != 1) {
    Serial.println("GPRMC aquisition failed!");
    Serial.print("Time taken : ");
    Serial.println(spanTime);
    Serial.println();
  }
}

//===================================================================//
//fetches the GSV header from the GPS outputs

void receiveGPGSV() {
  uint32_t spanTime = 0;
  byte byteBuffer = 0; // receive serial data byte
  uint8_t finishStatus = 0;  // indicates end of message
  uint8_t positionCount = 0;  // position counter
  uint8_t readFlag = 0;  // GPS flag
  uint8_t commaCount = 0;  // comma counter
  uint8_t satelliteBufferCount = 0;

  Serial.println("Fetching GPGSV data..");
  Serial.println();

  awake_time = millis();
  // ESP.wdtDisable();

  while ((finishStatus == 0) && (spanTime < 100)) {
    updateTime(); //keep the time ticking while fetching GPS data
    while ((gpsSerial.available() > 0) && (finishStatus == 0)) { // Check GPS data
      byteBuffer = gpsSerial.read();
      readFlag = 1;

      if (byteBuffer == '$' && positionCount == 0) positionCount = 1;
      if (byteBuffer == 'G' && positionCount == 1) positionCount = 2;
      if (byteBuffer == 'P' && positionCount == 2) positionCount = 3;
      if (byteBuffer == 'G' && positionCount == 3) positionCount = 4;
      if (byteBuffer == 'S' && positionCount == 4) positionCount = 5;
      if (byteBuffer == 'V' && positionCount == 5) positionCount = 6;

      if (positionCount == 6 && byteBuffer == ',') { // count commas in message
        commaCount++;
        readFlag = 0;
      }

      if (commaCount == 3 && readFlag == 1) {
        satelliteCountBuffer[satelliteBufferCount++] =  byteBuffer; // Speed in knot
        readFlag = 0;
      }

      if (commaCount >= 3) { // end of GSV message (GSV length can be variable)
        satelliteCountBuffer[satelliteBufferCount] = 0;
        Serial.println("GPGSV aquisition success");
        Serial.println();
        Serial.print("GPGSV Acquisition Time: ");
        Serial.println(spanTime);
        Serial.println();
        commaCount = 0;
        readFlag = 0;
        finishStatus = 1;
        positionCount = 0;
        satelliteBufferCount = 0;
      }
    }
    spanTime = millis() - awake_time;
  }
  if(finishStatus != 1) {
    Serial.println("GPGSV aquisition failed!");
    Serial.print("Time taken : ");
    Serial.println(spanTime);
    Serial.println();
  }
}

//===================================================================//
//initializes the GPS module and update the configuration

void initializeGPS () {
  //$PMTK220,100*2F - 10 Hz update rate
  //$PMTK220,1000*1F - 1Hz
  //$PMTK251,115200*1F - 115200 baud rate
  //$PMTK251,38400*27 - 38400 baud rate
  //$PMTK314,0,1,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2C - RMC, GSV

  Serial.println("Initializing GPS..");
  pinMode(GPS_RX, INPUT); //RX pin
  pinMode(GPS_TX, OUTPUT); //TX pin
  Serial.println("Starting with default baud rate (9600)..");
  gpsSerial.begin(9600);//default baud rate for GPS - will change this later
  // delay(10);
  // Serial.println("Configuring GPS..");
  // gpsSerial.println("$PMTK314,0,1,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0*2C"); //select the desired output sentences - RMC and GSV here
  // gpsSerial.flush();
  // gpsSerial.println("$PMTK251,38400*27"); //update the baud rate
  // gpsSerial.flush();
  // Serial.println("Ending com at default speed..");
  // Serial.println();
  // gpsSerial.end(); //end the current session to switch to new baud rate
  // delay(50);
  // gpsSerial.begin(38400); //begin serial with new baud rate (to achieve higher update rate)
  // delay(100);
  // Serial.println("Serial with new baud rate initialized..");
  // gpsSerial.println("$PMTK220,100*2F"); //change the update rate to 10Hz (to use 10Hz rate, we need higher baud rate)
  // gpsSerial.flush();
  if(gpsSerial.read() > 0)
  Serial.println("GPS initialization complete!");
  else Serial.println("GPS initialization failed!");
  Serial.println();
}

//===================================================================//

void getAltitude () {
  double absAltitudeValue, relAltitudeValue;
  Serial.println("Fetching altitude..");

  if(sensorInitialized) {
    // fetchDPS310Data(); //fetch values from sensor
    fetchBMP180Data();

    // absAltitudeValue = (double) (44330.0 * (1.0 - pow((pressureValue / SEA_LEVEL_PRESSURE), 0.190294))); //find altitude in metres (pressure value is in kPa)
    // relAltitudeValue = (double) (44330.0 * (1.0 - pow((pressureValue / baselinePressure), 0.190294)));

    absAltitudeValue = bmp180sensor.altitude(pressureValue/100, (SEA_LEVEL_PRESSURE)/100);
    relAltitudeValue = absAltitudeValue - baselineAltitude;
    Serial.print("Abs Altitude value = ");
    Serial.println(absAltitudeValue);
    Serial.print("Rel Altitude value = ");
    Serial.println(relAltitudeValue);
    Serial.println();

    absAltitudeString = "ABS: " + String(absAltitudeValue) + string_altitudeUnit; //format absAltitudeString
    relAltitudeString = "REL: " + String(relAltitudeValue) + string_altitudeUnit; //format relAltitudeString
    if(relAltitudeMode) altitudeString = string_altitude + String(relAltitudeValue) + string_altitudeUnit + " " + String(char(18)); //add the relative mode indicator at the end
    else altitudeString = string_altitude + String(absAltitudeValue) + string_altitudeUnit + " " + String(char(18)); //default altitude value
  }
  else {
    Serial.println("Pressure sensor was not initialized!");
    Serial.println();
  }
}

//===================================================================//

void initializeBMP180 () {
  if(bmp180sensor.begin()) {
    Serial.println("BMP180 initialization success.");
    Serial.println();
    sensorInitialized = true;

    baselinePressure = getBMPPressure();
    baselineAltitude = bmp180sensor.altitude(baselinePressure/100,(SEA_LEVEL_PRESSURE)/100);

    Serial.print("Baseline Pressure = ");
    Serial.print(baselinePressure);
    Serial.println(" Pa");

    Serial.print("Baseline Altitude = ");
    Serial.print(baselineAltitude);
    Serial.println(" m");
  }
  else {
    Serial.println("BMP180 initialization failed!");
    sensorInitialized = false;
  }
}

//===================================================================//

double getBMPPressure() {
  char bmpRet;

  bmpRet = bmp180sensor.startTemperature();

  if (bmpRet != 0) {
    delay(bmpRet);
    bmpRet = bmp180sensor.getTemperature(bmpTemperature);

    if (bmpRet != 0) {
      bmpRet = bmp180sensor.startPressure(3);

      if (bmpRet != 0) {
        delay(bmpRet);
        bmpRet = bmp180sensor.getPressure(bmpPressure,bmpTemperature);

        if (bmpRet != 0) {
          return(bmpPressure * 100); //convert mB to Pa
        }
        else Serial.println("BMP180 - Error retrieving pressure!\n");
      }
      else Serial.println("BMP180 - Error starting pressure!\n");
    }
    else Serial.println("BMP180 - Error retrieving temperature!\n");
  }
  else Serial.println("BMP180 - Error starting temperature\n");
}

//===================================================================//

void fetchBMP180Data () {
  if(sensorInitialized) {
    pressureValue = getBMPPressure(); //in Pascals
    temperatureValue = bmpTemperature;

    Serial.print("Temperature = ");
    Serial.print(temperatureValue);
    Serial.println(" Degrees Celsius");

    temperatureString = string_temperature + String(temperatureValue) + string_celsius; //format the string for displaying

    Serial.print("Pressure = ");
    Serial.print(pressureValue);
    Serial.println(" Pa");
    Serial.println();

    pressureString = string_pressure + String(pressureValue/1000.0) + string_pressureUnit; //format the string for displaying
  }
  else {
    Serial.println("BMP180 was not initialized!");
    Serial.println();
  }
}

//===================================================================//
//initializes the DPS310 sensor

// void initializeDPS310() {
//   Serial.println("Initilizing DPS310..");
//   ifxDps310.begin(Wire, 0x76);
//   //ifxDps310.begin(Wire); //with default address
//
//   ifxDps310.correctTemp();
//   Serial.println("Correcting temperature..");
//   // Serial.println(ifxDps310.startMeasureBothCont(4, 4, 4, 4)) //try measuring
//   Serial.println("Starting test measurement..");
//   int dpsRet = ifxDps310.startMeasureBothCont(3, 5, 3, 5);
//
//   if (dpsRet != 0) { //if the first try fails
//     sensorInitialized = false;
//     Serial.print("DPS310 initialization failed! : dpsRet = ");
//     Serial.println(dpsRet);
//     Serial.println();
//   }
//   else {
//     sensorInitialized = true;
//     Serial.println("DPS310 initialization success!"); //otherwise success
//     Serial.println();
//     fetchDPS310Data();
//     baselinePressure = pressureValue; //pressure in Pascals
//     baselineAltitude = (double) (44330.0 * (1.0 - pow((baselinePressure / SEA_LEVEL_PRESSURE), 0.190294)));
//   }
// }

//===================================================================//
//fetches temperature and pressutre data from DPS310 and updates
//the data arrays

// void fetchDPS310Data () {
//   if(sensorInitialized) {
//     Serial.println("Fetching DPS310 data..");
//     int dpsRet = ifxDps310.getContResults(temperature, temperatureCount, pressure, pressureCount);
//
//     if(dpsRet != 0) {
//       Serial.print("DPS310 fetching failed : dpsRet ");
//       Serial.println(dpsRet);
//       Serial.println();
//     }
//     else {
//       // Serial.println();
//       // Serial.println();
//       // Serial.print(temperatureCount); //this is variable
//       // Serial.println(" temperature values found: ");
//       for (uint8_t i = 0; i < temperatureCount; i++) {
//         // Serial.print(temperature[i]); //there'll be multiple measurements
//         // Serial.println(" degrees of Celsius");
//         temperatureValue += temperature[i]; //calculate average
//       }
//
//       if(temperatureCount == 0) temperatureValue = 0;
//       else temperatureValue = (double) temperatureValue / temperatureCount;
//
//       Serial.print("Temperature = ");
//       Serial.print(temperatureValue);
//       Serial.println(" Degrees Celsius");
//
//       temperatureString = string_temperature + String(temperatureValue) + String_celsius; //format the string for displaying
//       // Serial.println();
//       // Serial.print(pressureCount); //this is variable
//       // Serial.println(" pressure values found: ");
//       for (uint8_t i = 0; i < pressureCount; i++) {
//         // Serial.print(pressure[i]); //multiple measurements
//         // Serial.println(" Pascal");
//         pressureValue += pressure[i]; //finds the sum
//       }
//
//       if(pressureCount == 0) pressureValue = 0;
//       else pressureValue = (double) pressureValue / pressureCount; //find average
//
//       Serial.print("Pressure = ");
//       Serial.print(pressureValue);
//       Serial.println(" Pa");
//       Serial.println();
//
//       // pressureValue /= 100.0; //convert the Pa reading to kPa
//       pressureString = string_pressure + String(pressureValue/100.0) + string_pressureUnit; //format the string for displaying
//     }
//
//     //Wait some time, so that the Dps310 can refill its buffer
//     delay(10);
//   }
//   else {
//     Serial.println("DPS310 was not initialized!");
//     Serial.println();
//   }
// }

//===================================================================//
//updates aligned text - clears the text area first and then writes
//the new text without affecting any other parts of the display

void updateAlignedText (String s, int y, int align) { //x is locally calculated for aligned text
  if((s.length() < 15) && (y > -1) && (y < 48)) { //bound checking
    clearDisplaySection(0, y, DISPLAY_WIDTH, DISPLAY_HEIGHT); //clear the line first
    if(align == ALIGN_CENTER) { //align center
      LCD_5110.setCursor((int)((DISPLAY_WIDTH-((s.length() * MONO_WIDTH) + (s.length()-1)))/2), y); //calculate the x
      //Serial.println(s.length());
      //Serial.println(((DISPLAY_WIDTH-(s.length() * MONO_WIDTH))/2));
      LCD_5110.print(s);
      // UPDATE_DISPLAY;
    }
  }
}

//===================================================================//
//updates non-aligned text - clears the text area first and then writes
//the new text without affecting any other parts of the display

void updateText (String s, uint8_t x, uint8_t y) {
  if((s.length() < 15) && (y > -1) && (y < 48)) { //bound checking
    clearDisplaySection(x, y, s.length()); //clear the area taken up by the text
    LCD_5110.setCursor(x, y);
    LCD_5110.print(s);
    // UPDATE_DISPLAY;
  }
}

//===================================================================//
//clears only a specific area of the display occupied by text (text area)

void clearDisplaySection(uint8_t x, uint8_t y, uint8_t length) { //length is the length of string
  for(uint8_t i=0; i < MONO_HEIGHT; i++) { //draw white lines for the height of a char
    LCD_5110.drawFastHLine(x, y+i, ((MONO_WIDTH * length) + (length -1)), WHITE); //note: each char has one vertical line space
  }
}

//===================================================================//
//clears only a specific rectangular area of the display

void clearDisplaySection(uint8_t x, uint8_t y, uint8_t width, uint8_t height) { //length is the length of string
  for(uint8_t i=0; i < height; i++) { //just fill the area with white horizontal lines
    LCD_5110.drawFastHLine(x, y+i, width , WHITE);
  }
}

//===================================================================//
//prints non-aligned text without clearing text area, and updates display
//use updateText() for refreshing specific text

void printText (String s, int x, int y) {
  if((s.length() < 15) && (y > -1) && (y < 48)) {
    LCD_5110.setCursor(x, y);
    LCD_5110.print(s);
    UPDATE_DISPLAY;
  }
}

//===================================================================//
//prints aligned text on top (without clearing) and updates the display

void printAlignedText (String s, int y, int a) {
  if((s.length() < 15) && (y > -1) && (y < 48)) {
    if(a == ALIGN_CENTER) { //align center
      LCD_5110.setCursor((int)((DISPLAY_WIDTH-((s.length() * MONO_WIDTH) + (s.length()-1)))/2), y);
      LCD_5110.print(s);
    }
  }
}

//===================================================================//
//prints text with align after clearing the entire display

void printAlignedTextAfterClear(String s, uint8_t y, uint8_t align) {
  CLEAR_DISPLAY;
  LCD_5110.setTextSize(1);
  LCD_5110.setTextColor(BLACK);

  updateAlignedText(s, y, align); //aligned text
  UPDATE_DISPLAY;
}

//===================================================================//
//prints text w/t align after clearing the entire display

void printTextAfterClear(String s, uint8_t x, uint8_t y) {
  CLEAR_DISPLAY;
  LCD_5110.setTextSize(1);
  LCD_5110.setTextColor(BLACK);

  updateText(s, x, y); //non-aligned text
  UPDATE_DISPLAY;
}

//===================================================================//
