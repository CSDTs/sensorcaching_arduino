#include <SD.h>
#include <Wire.h>
#include "RTClib.h"

/**************************************************************************************
SENSORCACHE ARDUINO PROGRAM
VERSION 0.1
CONTACT:  CBSHING@GMAIL.COM
*************************************************************************************/

/*
  This Arduino project uses a Bluetooth serial modem, Adafruit Datalogger Shield (v2), and Arduino Uno
  to record sensor data and send this file to a bluetooth phone
  
  There is very little that is novel in the code.  Much is comprised by taking other people's existing
  work to allow for the Arduino and Android to talk with each other.  This is created based on code
  written by Lady Ada ( ) from Adafruit and Google Bluetooth Chat.  This code is meant to be part of
  Culturally Situated Sensors - Sensorcaching project.  Anyone is free to download and modify as needed.
  However, if you would like to associate your sensor with the Culturally Situated Sensing network, please
  go to xxx.edu and sign up.
  
  Instructions for this project:
  1. Go to the Culturally Situated Sensors website (xxx.edu) and download Sensorcache sensor instructions,
     fill out the necessary forms, and download this Sensorcache sensor code
  2. Hook up your sensorcache as seen in the instructions and upload this code to your sensor.
  3. Download "Sensorcaching" from the Android Market
  4. Open up Sensorcaching app on your phone and click the "Share" Button.  
  5. Select the "Test my Sensor" Option.  To pair with your sensor select "Connect a Device", then select "Firefly".  
     Now you have paired Android to Arduino.  You may need to type passcode: 1234.
  6. Information will automatically be 
     
  
  Parts Lists:
  
  Software:
  
  
*/

//User defined variables
//#define FIRMWARE_VERSION  0.1
const int sensorcacheid = 1;
//const char sensortype[] = "sensor0";
float sensorValue = 0;
//float sensorMultiplier = 1;
float sensorOffset = 0;

//For testing purposes
const char sensortype[] = "light";
float sensorMultiplier = 1024/5;

// how many milliseconds before checking if the button is pressed. 1000 ms is once a second
#define BUTTON_INTERVAL  100 // mills between entries (reduce to take more/faster data)

// how many milliseconds between grabbing data and logging it. 1000 ms is once a second
#define LOG_INTERVAL  1000 // mills between entries (reduce to take more/faster data)
uint32_t LogTime = 0; // time of last log

// how many milliseconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set it to 10*LOG_INTERVAL to write all data every 10 datareads, you could lose up to 
// the last 10 reads if power is lost but it uses less power and is much faster!
#define SYNC_INTERVAL 10000 // mills between calls to flush() - to write data to the card
uint32_t syncTime = 0; // time of last sync()

#define DEBUG_TO_SERIAL   1 // print debug serial lines

// the digital pins that connect to the LEDs
#define redLEDpin 2
#define greenLEDpin 3

// the analog pins that connect to the sensors
#define batteryPin 0          // analog 0
#define sensorPin 1           // analog 1

// the battery reference calculations
const int ref_voltage = 5;
//const float ref_voltage = 3.3;
float batteryVoltage = ref_voltage;

RTC_DS1307 RTC; // define the Real Time Clock object

// for the bluetooth connection state, we use digital pin 6
const int BTconnectPin = 6;
int BTconnectstate = 0;
// for the button, we use digital pin 7
const int buttonPin = 7;
int buttonstate = 0;
// for the data logging shield, we use digital pin 10 for the SD cs line
const int chipSelect = 10;

File logfile;
char filename1[] = "LOGFILE1.txt";
char filename2[] = "LOGFILE2.txt";
boolean filename_flag = false;

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  digitalWrite(redLEDpin, HIGH);
  
  while(1);
}

void setup(void)
{
  Serial.begin(115200);
  Serial.println();
  
  // use debugging LEDs
  pinMode(redLEDpin, OUTPUT);
  pinMode(greenLEDpin, OUTPUT);
  
  //define button & BT state pins
  pinMode(BTconnectPin, INPUT);
  pinMode(buttonPin, INPUT);
  
  // make sure that the default chip select pin is set to
  // output, even if you don't use it
  // for the SD logger:
  pinMode(10, OUTPUT);
  
  Serial.print("Initializing SD card....");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    error("Card failed, or not present");
  }
  
  Serial.println("card initialized.");
  
   // connect to RTC
  Wire.begin();  
  if (!RTC.begin()) {
    logfile.println("RTC failed");
    Serial.println("RTC failed");
  }
  
  RTC.adjust(DateTime(__DATE__,__TIME__));
  
  SD_init();
}

void loop(void)
{
  DateTime now;

  // delay for the amount of time we want between readings
  delay((BUTTON_INTERVAL -1) - (millis() % BUTTON_INTERVAL));
  
  buttonstate = digitalRead(buttonPin);
  if(buttonstate == HIGH) {
#if DEBUG_TO_SERIAL
  Serial.println("Button Pressed");
#endif // DEBUG_TO_SERIAL     
    logfile.close();
    connectBT();
    SD_init();
  }

  // delay for the amount of time we want between readings
  if ((millis() - LogTime) < LOG_INTERVAL) return;
  LogTime = millis();
  
  digitalWrite(greenLEDpin, HIGH);
 
  // print the id number   
  logfile.print(sensorcacheid);
  logfile.print(",");
#if DEBUG_TO_SERIAL
  Serial.print(sensorcacheid);
  Serial.print(",");
#endif // DEBUG_TO_SERIAL

  // fetch the time
  now = RTC.now();
  // log time
  logfile.print('"');
  logfile.print(now.year(), DEC);
  logfile.print("/");
  logfile.print(now.month(), DEC);
  logfile.print("/");
  logfile.print(now.day(), DEC);
  logfile.print(" - ");
  logfile.print(now.hour(), DEC);
  logfile.print(":");
  logfile.print(now.minute(), DEC);
  logfile.print('"');
#if DEBUG_TO_SERIAL
  Serial.print('"');
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print(" - ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  Serial.print(now.minute(), DEC);
  Serial.print('"');
#endif //DEBUG_TO_SERIAL

  analogRead(sensorPin);
  delay(10); 
  int sensorReading = analogRead(sensorPin);  
  float sensorVoltage = (sensorReading * ref_voltage)/1024;
  sensorValue = (sensorVoltage*sensorMultiplier) + sensorOffset;
  
  logfile.print(", ");    
  logfile.print(sensorValue);
#if DEBUG_TO_SERIAL
  Serial.print(", ");   
  Serial.print(sensorValue);
#endif // DEBUG_TO_SERIAL

  analogRead(batteryPin);
  delay(10);
  int batteryReading = analogRead(batteryPin);
  batteryVoltage = (batteryReading*ref_voltage)/1024;
  
  logfile.print(", ");    
  logfile.print(batteryVoltage);
#if DEBUG_TO_SERIAL
  Serial.print(", ");   
  Serial.print(batteryVoltage);
#endif // DEBUG_TO_SERIAL
  
  logfile.println();
#if DEBUG_TO_SERIAL
  Serial.println();
#endif // DEBUG_TO_SERIAL

  digitalWrite(greenLEDpin, LOW);

  // Now we write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
  // which uses a bunch of power and takes time
  if ((millis() - syncTime) < SYNC_INTERVAL) return;
  syncTime = millis();
  
  // blink LED to show we are syncing data to the card & updating FAT!
  digitalWrite(redLEDpin, HIGH);
  logfile.flush();
  digitalWrite(redLEDpin, LOW);
  
}

void SD_init(void)
{  
  // create a new file
  if (!filename_flag) {
    SD.remove(filename2);
    logfile = SD.open(filename2, FILE_WRITE); 
    filename_flag = true;
  }
  else {
    SD.remove(filename1);
    logfile = SD.open(filename1, FILE_WRITE); 
    filename_flag = false;
  }

#if DEBUG_TO_SERIAL
  Serial.print("Filename_flag = ");
  Serial.println(filename_flag);
#endif // DEBUG_TO_SERIAL  

    //For up to 99 different files use the following
/*  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      logfile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
 */ 
  if (! logfile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  if(filename_flag)
    Serial.println(filename2);
  else
    Serial.println(filename1);
  
  logfile.print("id,timestamp,");
  logfile.print(sensortype);
  logfile.println(",bat");
#if DEBUG_TO_SERIAL
  Serial.print("id,timestamp,");
  Serial.print(sensortype);
  Serial.println(",bat");  
#endif //DEBUG_TO_SERIAL
  
  return;
}

void connectBT(void)
{
  delay(10);
  //Wait for BT to be connected.
  char c;
  while(c != '$') {
    if(Serial.available())
      c = Serial.read();
#if DEBUG_TO_SERIAL
  //Serial.println("Waiting for a connection 1...");
#endif //DEBUG_TO_SERIAL
  }
  while(c != '#') {
    if(Serial.available())
      c = Serial.read();
#if DEBUG_TO_SERIAL
  //Serial.println("Waiting for a connection 2...");
#endif //DEBUG_TO_SERIAL
  }
/*  
  while(!BTconnectstate == HIGH){
    BTconnectstate = digitalRead(BTconnectPin);
#if DEBUG_TO_SERIAL
  Serial.print("Waiting for a connection...");
#endif //DEBUG_TO_SERIAL    
  }
*/

  //This statement will only be shown during the test phase 
  Serial.print("$$ You have connected to Sensorcache Sensor ");  
  Serial.print(sensorcacheid);
  Serial.println(" %%");
  
  // re-open the file for reading:
  if(filename_flag)
    logfile = SD.open(filename2);
  else
    logfile = SD.open(filename1);
  
  if(logfile)  {
    Serial.print("&& ");
    int16_t c;
    while ((c = logfile.read()) > 0) {
      Serial.print((char)c);
    }
    logfile.close();  
    Serial.print(" %%");
  }
  else {
    error("&& File not found %%");
  }

/*  
  while(BTconnectstate == HIGH) {
    BTconnectstate = digitalRead(BTconnectPin);
#if DEBUG_TO_SERIAL
  Serial.print("Waiting to end a connection...");
#endif //DEBUG_TO_SERIAL   
  }
*/  
  return;
}
