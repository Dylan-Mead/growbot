/*
  SD card datalogger

  This example shows how to log data from three analog sensors
  to an SD card using the SD library.

  The circuit:
   analog sensors on analog ins 0, 1, and 2
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

  created  24 Nov 2010
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

*/
//SD
#include <SPI.h>
#include <SD.h>
const int chipSelect = 10;

//for Lux sensor
#include <Wire.h>
#include <SparkFun_VEML7700_Arduino_Library.h> // Click to get library: http://librarymanager/All#SparkFun_VEML7700
VEML7700 luxSensor; // Create a VEML7700 object

//for LCD
//#include <Wire.h>
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD
SerLCD lcd; // Initialize the library with default I2C address 0x72

//for temp,hum,pres sensor
//#include <Wire.h>
//#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme; // I2C

//for RTC
#include "RTClib.h"
RTC_PCF8523 rtc;

//environmental conditions and controllers constants
const byte heater = 7;
const byte vent = 3;
const byte l0 = 6;
const byte l1 = 5;
const byte l2 = 4;
const float min_temps[] = {36,38,41,43,46,48,48,48,46,43,38,36};
const float min_lux[] = {15000, 35000, 50000};
const float max_temp = 90;
const float min_photo_temp = 42;
const float max_humidity = 90;

//delays
const int main_delay = 10000;
const int display_delay = 3000;




void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  //set up lux sensor
  Wire.begin();
  //mySensor.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  // Begin the VEML7700 using the Wire I2C port
  // .begin will return true on success, or false on failure to communicate
  if (luxSensor.begin() == false)
  {
    Serial.println(F("Unable to communicate with the VEML7700. Please check the wiring. Freezing..."));
    while (1);
  }

  //lcd
  //Wire.begin();
  lcd.begin(Wire); //Set up the LCD for I2C communication
  lcd.setBacklight(255, 255, 255); //Set backlight to bright white
  lcd.setContrast(5); //Set contrast. Lower to 0 for higher contrast.
  lcd.clear(); //Clear the display - this moves the cursor to home position as well

  //temp, pres, hum
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
  }
  rtc.start();
  float drift = 43; // seconds plus or minus over oservation period - set to 0 to cancel previous calibration.
  float period_sec = (7 * 86400);  // total obsevation period in seconds (86400 = seconds in 1 day:  7 days = (7 * 86400) seconds )
  float deviation_ppm = (drift / period_sec * 1000000); //  deviation in parts per million (μs)
  float drift_unit = 4.34; // use with offset mode PCF8523_TwoHours
  // float drift_unit = 4.069; //For corrections every min the drift_unit is 4.069 ppm (use with offset mode PCF8523_OneMinute)
  int offset = round(deviation_ppm / drift_unit);
  // rtc.calibrate(PCF8523_TwoHours, offset); // Un-comment to perform calibration once drift (seconds) and observation period (seconds) are correct
  // rtc.calibrate(PCF8523_TwoHours, 0); // Un-comment to cancel previous calibration
  Serial.print(F("Offset is ")); Serial.println(offset); // Print to control offset

  //relay
  pinMode(heater, OUTPUT);
  pinMode(l0, OUTPUT);
  pinMode(l1, OUTPUT);
  pinMode(l2, OUTPUT);
  pinMode(vent, OUTPUT);
}

void loop() {
  float lux = luxSensor.getLux();
  float temp = bme.readTemperature() * 1.8 + 32;
  float humidity = bme.readHumidity();
  DateTime now = rtc.now();


  printToLcd(lux, temp, humidity, now);
  //envControl(lux, temp, now);
  printToSd(lux, temp, humidity, now);
  envControl(lux, temp, humidity, now);

//  delay(2000);
}

void printToLcd(float lux, float temp, float humidity, DateTime now){
  //
  lcd.clear();
  lcd.setCursor(0, 0);//column 0, row 1 (zero indexed)
  lcd.print(F("Lux:"));
  lcd.setCursor(0, 1);
  lcd.print(String(lux));
  delay(display_delay);

  lcd.clear();
  lcd.setCursor(0, 0);//column 0, row 1 (zero indexed)
  lcd.print(F("Rh%"));
  lcd.setCursor(0, 1);
  lcd.print(String(humidity));
  delay(display_delay);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.println(String(now.year())+'/'+String(now.month())+'/'+String(now.day()));
  lcd.setCursor(0, 1);
  lcd.println(String(now.hour())+':'+String(now.minute())+':'+String(now.second()));
  delay(display_delay);

  lcd.clear();
  lcd.setCursor(0, 0);//column 0, row 1 (zero indexed)
  lcd.print(F("Temp"));
  lcd.setCursor(0, 1);
  lcd.print(String(temp));
  delay(display_delay);

}

void printToSd(float lux, float temp, float humidity, DateTime now){
  //
  String dataString = "";
  dataString += String(now.year())+'/'+String(now.month())+'/'+String(now.day())+","+String(now.hour())+':'+String(now.minute())+':'+String(now.second())+","+String(lux)+","+String(temp)+","+String(humidity);
  File dataFile = SD.open("datalog2.txt", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println(F("error sd datalog2.txt"));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.println(F("Error logging SD"));
    lcd.setCursor(0, 1);
    lcd.println(F("datalog2.txt"));
  }
  delay(main_delay);
}

void envControl(float lux, float temp, float hum, DateTime now){
  //Serial.println(F("lux min_lux[0] min_lux[1] min_lux[2]"));
  //Serial.println(String(lux)+","+String(min_lux[0])+","+String(min_lux[1])+","+String(min_lux[2]));
  
  //light control
  if (temp < min_photo_temp){
    Serial.println(F("temp < min_photo_temp"));
    digitalWrite(l0, LOW);
    digitalWrite(l1, LOW);
    digitalWrite(l2, LOW);
  }
  else{
    if (now.hour() > 6 && now.hour() < 19){// start and stop time for lights
      if (lux < min_lux[0]){
        digitalWrite(l0, HIGH);
        if (lux < min_lux[1]){
          digitalWrite(l1, HIGH);
          if (lux < min_lux[2]){
            digitalWrite(l2, HIGH);
          } else{
            digitalWrite(l2, LOW);
          }
        } else{
          digitalWrite(l1, LOW);
        }
      } else{
        digitalWrite(l0, LOW);
      }
    } else{
      Serial.println(F("!(now.hour() > 6 && now.hour() < 18)"));
      digitalWrite(l0, LOW);
      digitalWrite(l1, LOW);
      digitalWrite(l2, LOW);
    }
  }

  //heat control
  if (temp < min_temps[now.month() - 1]){
    digitalWrite(heater, HIGH);
  } else {
    digitalWrite(heater, LOW);
  }

  //vent control
  if ((temp > max_temp) || (hum > max_humidity)){
    digitalWrite(vent, HIGH);
  } else {
    digitalWrite(vent, LOW);
  }
  /*
  //test control
  if (1<2){
    digitalWrite(vent, HIGH);
  } else {
    digitalWrite(vent, LOW);
  }*/

  delay(main_delay);
}

bool light_on(float lux, float temp, float hum, DateTime now){
  bool on = false;
  if ((now.hour() > 6 && now.hour() < 18) && (temp > min_photo_temp)){
    on = true;
  }
  else{
    on = false;
  }
  return on;
}