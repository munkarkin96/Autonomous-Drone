#include <TinyGPS++.h>
#include <SoftwareSerial.h>
/*
 This example uses software serial and the TinyGPS++ library by Mikal Hart
 Based on TinyGPSPlus/DeviceExample.ino by Mikal Hart
 Modified by acavis
*/

// Choose two Arduino pins to use for software serial
// The GPS Shield uses D2 and D3 by default when in DLINE mode
int RXPin = 2;
int TXPin = 3;
int loraRx = 10;
int loraTx = 11; 

int ledPin = 7; 

// The Skytaq EM-506 GPS module included in the GPS Shield Kit
// uses 4800 baud by default
int GPSBaud = 4800;

// Create a TinyGPS++ object called "gps"
TinyGPSPlus gps;

// Create a software serial port called "gpsSerial"
SoftwareSerial gpsSerial(RXPin, TXPin);
SoftwareSerial mySerial(loraRx, loraTx);

void setup()
{
  // Start the Arduino hardware serial port at 9600 baud
  Serial.begin(9600);
  mySerial.begin(9600);

  // Start the software serial port at the GPS's default baud
  gpsSerial.begin(GPSBaud);

  Serial.println(F("DeviceExample.ino"));
  Serial.println(F("A simple demonstration of TinyGPS++ with an attached GPS module"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  pinMode(ledPin, OUTPUT);
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      displayInfo();

  // If 5000 milliseconds pass and there are no characters coming in
  // over the software serial port, show a "No GPS detected" error
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected"));
    while(true);
  }
}

void displayInfo()
{
  Serial.println(F("Location: ")); 
  if (gps.location.isValid())
  {
    /*
    Serial.print(gps.location.lat(),6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(),6);
*/  

    //Serial.println("GPS Fixed");

    //delay(100);

    digitalWrite(ledPin, HIGH);

    mySerial.print("Latitude: ");
    String input1 = Serial.readString();
    //delay(50);
    mySerial.println(gps.location.lat(),8);
    String inputLatitude = Serial.readString();
    //inputLatitude = "Lat: " + inputLatitude; 
    //mySerial.println(inputLatitude);

    //delay(100);
    mySerial.print("Longitude: ");
    String input2 = Serial.readString();
    mySerial.println(gps.location.lng(),8);
    String inputLongitude = Serial.readString(); 
    //inputLongitude = "Long: " + inputLongitude; 
    //mySerial.println(inputLongitude);
    
    //mySerial.println(inputLatitude + ',' + inputLongitude);
    //mySerial.println((gps.location.lat(),8) + ", " + (gps.location.lat(),8));
    delay(100);


  }
  else
  {

    digitalWrite(ledPin, LOW);
    Serial.println(F("INVALID"));
    
    mySerial.print("INVALID");
    String inputInvalid = Serial.readString(); 
    mySerial.print(inputInvalid);
  }
/*
  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();*/
}
