#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = 12; // Connects to module's RX 
static const uint8_t PIN_MP3_RX = 13; // Connects to module's TX 
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

// Create the Player object
DFRobotDFPlayerMini player;

void setup() {

  // Init USB serial port for debugging
  Serial.begin(9600);
  // Init serial port for DFPlayer Mini
  softwareSerial.begin(9600);

  // Start communication with DFPlayer Mini
  if (player.begin(softwareSerial)) {
   Serial.println("OK");

    // Set volume to maximum (0 to 30).
    player.volume(30);
    // Play the first MP3 file on the SD card
    player.loop(1);
  } else {
    Serial.println("Connecting to DFPlayer Mini failed!");
  }
  pinMode( 1, INPUT_PULLUP );
  pinMode( 2, OUTPUT );
}

void loop() {
  digitalWrite( 2, digitalRead( 1 ) );
}
