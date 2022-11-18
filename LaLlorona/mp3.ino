
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

SoftwareSerial mySoftwareSerial(10, 11); // RX, TX - pins 
// Creating the Player object
DFRobotDFPlayerMini speaker;


void setup() {
   // Init USB serial port for debugging
  Serial.begin(9600);
  // Init serial port for DFPlayer Mini
  softwareSerial.begin(9600);

}

void loop() {
// code for when - players move different amounts --> channge volume 
 // Set volume to maximum - when closest 
    speaker.volume(30);
    
    // Play the first MP3 file on the SD card - save mp3 file as 0001.mp3 for program to read it   
    speaker.play(1);

}
