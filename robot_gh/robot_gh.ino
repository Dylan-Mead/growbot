/*
  Environmental Monitor and Logger

  We will read from a couple sensors and a clock
  and write the output to lcd, and log file on sd card

  The circuit:
   Adafruit Adalogger featherwing
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 10 (for MKRZero SD: SDCARD_SS_PIN)
 ** Adalogger 3.3V - Arduino Power 3.3V
 ** Adalogger GND - Arduino Power GND

   LCD 16x2 display:
** QWIIC - QWIIC

   Adafruit Adalogger featherwing
   BAttery Powered RTC as follows:
 ** SDA - QWIIC SDA (blue)
 ** SCL - QWIIC SCL (yellow)
 ** Adalogger 3.3V - Arduino Power 3.3V
 ** Adalogger GND - Arduino Power GND

   SPARKFUN BME280 Temp, Humidity, Pressure sensor:
**  QWIIC - QWIIC

   SPARKFUN VEML7700 Lux sensor:
**  QWIIC - QWIIC

  created  2023-06-13
  by Dylan Mead

*/

//Imports and static variables

//for SD card
#include <SPI.h>
#include <SD.h>
//const int chipSelect = 10;

//for LCD
#include <Wire.h>
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD
SerLCD lcd; // Initialize the library with default I2C address 0x72

//for RTC
#include "RTClib.h"
RTC_PCF8523 rtc;
//char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

//for temp,hum,pres sensor
#include <Wire.h>
//#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//#define BME_SCK 13
//#define BME_MISO 12
//#define BME_MOSI 11
//#define BME_CS 10
//#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
//unsigned long delayTime;

//for Lux sensor
#include <SparkFun_VEML7700_Arduino_Library.h> // Click to get library: http://librarymanager/All#SparkFun_VEML7700
VEML7700 mySensor; // Create a VEML7700 object


void setup() {
  // setup all devices and files
  Serial.begin(57600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

    //sd
    Serial.print(F("Initializing SD card..."));
    if (!SD.begin(10)) {//CS pin is 10 on Uno. see if the card is present and can be initialized
      Serial.println("Card failed, or not present");
      // don't do anything more:
      while (1);
    }
    Serial.println("card initialized.");

    //lcd
    Wire.begin();
    lcd.begin(Wire); //Set up the LCD for I2C communication
    lcd.setBacklight(255, 255, 255); //Set backlight to bright white
    lcd.setContrast(5); //Set contrast. Lower to 0 for higher contrast.
    lcd.clear(); //Clear the display - this moves the cursor to home position as well


    //rtc
    //Serial.begin(57600);
    #ifndef ESP8266
      while (!Serial); // wait for serial port to connect. Needed for native USB
    #endif
      if (! rtc.begin()) {
        Serial.println(F("Couldn't find RTC"));
        Serial.flush();
        while (1) delay(10);
      }
    
  if (! rtc.initialized() || rtc.lostPower()) {
    Serial.println(F("RTC is NOT initialized, let's set the time!"));
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    //
    // Note: allow 2 seconds after inserting battery or applying external power
    // without battery before calling adjust(). This gives the PCF8523's
    // crystal oscillator time to stabilize. If you call adjust() very quickly
    // after the RTC is powered, lostPower() may still return true.
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));

  // When the RTC was stopped and stays connected to the battery, it has
  // to be restarted by clearing the STOP bit. Let's do this to ensure
  // the RTC is running.
  rtc.start();
   // The PCF8523 can be calibrated for:
  //        - Aging adjustment
  //        - Temperature compensation
  //        - Accuracy tuning
  // The offset mode to use, once every two hours or once every minute.
  // The offset Offset value from -64 to +63. See the Application Note for calculation of offset values.
  // https://www.nxp.com/docs/en/application-note/AN11247.pdf
  // The deviation in parts per million can be calculated over a period of observation. Both the drift (which can be negative)
  // and the observation period must be in seconds. For accuracy the variation should be observed over about 1 week.
  // Note: any previous calibration should cancelled prior to any new observation period.
  // Example - RTC gaining 43 seconds in 1 week
  float drift = 43; // seconds plus or minus over oservation period - set to 0 to cancel previous calibration.
  float period_sec = (7 * 86400);  // total obsevation period in seconds (86400 = seconds in 1 day:  7 days = (7 * 86400) seconds )
  float deviation_ppm = (drift / period_sec * 1000000); //  deviation in parts per million (μs)
  float drift_unit = 4.34; // use with offset mode PCF8523_TwoHours
  // float drift_unit = 4.069; //For corrections every min the drift_unit is 4.069 ppm (use with offset mode PCF8523_OneMinute)
  int offset = round(deviation_ppm / drift_unit);
  // rtc.calibrate(PCF8523_TwoHours, offset); // Un-comment to perform calibration once drift (seconds) and observation period (seconds) are correct
  // rtc.calibrate(PCF8523_TwoHours, 0); // Un-comment to cancel previous calibration
  Serial.print(F("Offset is ")); Serial.println(offset); // Print to control offset

    //temp
    //Serial.begin(57600);
    //while(!Serial);    // time to get serial running
    //Serial.println(F("BME280 test"));

    unsigned status;
    
    // default settings
    status = bme.begin();  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println(F("Could not find a valid BME280 sensor, check wiring, address, sensor ID!"));
        Serial.print(F("SensorID was: 0x")); Serial.println(bme.sensorID(),16);
        Serial.print(F("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n"));
        Serial.print(F("   ID of 0x56-0x58 represents a BMP 280,\n"));
        Serial.print(F("        ID of 0x60 represents a BME 280.\n"));
        Serial.print(F("        ID of 0x61 represents a BME 680.\n"));
        while (1) delay(10);
    }
    Serial.println(F("-- Default Test --"));
    //delayTime = 5000;
    Serial.println();
    //lux
    Wire.begin();
    //mySensor.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
    // Begin the VEML7700 using the Wire I2C port
    // .begin will return true on success, or false on failure to communicate
    if (mySensor.begin() == false)
    {
      Serial.println(F("Unable to communicate with the VEML7700. Please check the wiring. Freezing..."));
      while (1);
  }

}

void loop() {
  // read from sensors
  
  // write to lcd
  printTimeLcd();
  printEnvLcd();
  printLuxLcd();


  // write to files
}

void printTimeLcd(){
  lcd.clear();
  DateTime now = rtc.now();
  Serial.print(String(now.year()));
  Serial.print(F("/"));
  Serial.print(String(now.month()));
  Serial.print(F("/"));
  Serial.println(String(now.day()));
  lcd.setCursor(0, 0);
  lcd.println(String(now.year())+'/'+String(now.month())+'/'+String(now.day()));
  Serial.print(String(now.hour()));
  Serial.print(F(":"));
  Serial.print(String(now.minute()));
  Serial.print(F(":"));
  Serial.println(String(now.second()));
  lcd.setCursor(0, 1);
  lcd.println(String(now.hour())+':'+String(now.minute())+':'+String(now.second()));
  delay(1000);
}

void printEnvLcd(){
  lcd.clear();
  float celcius = bme.readTemperature();
  lcd.setCursor(0, 0);//column 0, row 1 (zero indexed)
  Serial.print(celcius * 1.8 + 32);
  Serial.println(F(" F"));
  lcd.print(celcius * 1.8 + 32);
  lcd.println(F(" F"));
  lcd.setCursor(0, 1);//column 0, row 1 (zero indexed)
  float humidity = bme.readHumidity();
  Serial.print(humidity);
  Serial.println(F(" %Rh"));
  lcd.print(humidity);
  lcd.println(F(" %Rh"));
  delay(1000);
}

void printLuxLcd(){
  lcd.clear();
  float lux = mySensor.getLux();
  lcd.setCursor(0, 0);//column 0, row 1 (zero indexed)
  Serial.print(lux);
  Serial.println(F(" Lux"));
  lcd.print(lux);
  lcd.println(F(" Lux"));
  delay(1000);
}


