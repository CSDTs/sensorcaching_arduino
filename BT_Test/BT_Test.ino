#include <SD.h>

File myFile;

int lightPin = A0;
const int connectPin = 6;

int connectState = 0;
int lightValue = 0;
int counter = 0;


void setup(){
  Serial.begin(115200);
  Serial.print("Initializing SD card...");
  
  //Pin to determine if logger connected
  pinMode(connectPin, INPUT);
  //Pin for Logger
  pinMode(10, OUTPUT);
  
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  
  setupFile();
  
  Serial.println("Finished Setup");
}

void loop() {
  lightValue = analogRead(lightPin);
  Serial.println(counter);
  myFile.print(counter);
  myFile.print(",");
  myFile.print(lightValue);
  myFile.print(",");
  myFile.println("A,Z");
  connectState = digitalRead(connectPin);
  if(connectState == HIGH) {
    myFile.close();
    connectBT();
    while(connectState == HIGH);
    setupFile();
    
  }

  counter++;
  delay(1); 
}

void connectBT() {  
  // re-open the file for reading:
  myFile = SD.open("testfile.txt");
  if (myFile) {
    Serial.println("testfile.txt:");
    
    // read from the file until there's nothing else in it:
    while (myFile.available()) {
    	Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
    Serial.println("Finished Transfer File");
  } else {
  	// if the file didn't open, print an error:
    Serial.println("error opening test2.txt");
  }
}

void setupFile() {
  myFile = SD.open("testfile.txt", FILE_WRITE);
  
  //ok to open file
  if(!myFile) {
    myFile.println("Counter, Light, Blank, Blank2");
  }
  else { 
    Serial.println("error opening testfile.txt");
  }
}
  
