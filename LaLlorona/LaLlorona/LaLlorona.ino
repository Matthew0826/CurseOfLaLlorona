#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

//This is the code that runs La Llorona
const int sensors[6] = {A0, A1, A2, A3, A4, A5};
const int resistorValues[8] = {100, 220, 330, 470, 510, 680, 1000, 2000};
// Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = 11; // Connects to module's RX 
static const uint8_t PIN_MP3_RX = 10; // Connects to module's TX 
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);

// Create the Player object
DFRobotDFPlayerMini player;

const double coords[6][8][2] = {{{0.5, 1.5}, {0.5, 2.5}, {0.5, 3.5}, {0.5, 4.5}, {0.5, 5.5}, {0.5, 6.5}, {0.75, 7.5}, {1.75, 7.5} },
{{1.25, 1.25}, {2.25, 2.25}, {3.25, 3.25}, {3.75, 4}, {4.5, 4.5}, {3.75, 5.25}, {2.75, 5}, {2.5, 6.25} },
{{1.75, 0.5}, {3, 0.5}, {4.5, 0.5}, {5.5, 0.75}, {6.5, 0.75}, {7.5, 0.75}, {8.5, 0.75}, {8.5, 2} },
{{2.75, 8}, {3.75, 8.25}, {4.75, 8.75}, {5.5, 9.25}, {6.25, 8}, {7, 7}, {7.5, 6}, {6.5, 5.75} },
{{5.5, 5.25}, {5.5, 4.5}, {6.5, 4}, {7.5, 3.5}, {8.5, 3.5}, {8.5, 4.5}, {8.5, 5.5}, {8.5, 6.5} },
{{8.75, 7.5}, {9, 8.25}}};

//This records where each piece is
//The first number is the line it is on, the second number is which space
int pieces[4][2] = {{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}};
double distances[4];
int laLloronaCoords[2];
int currentTurn = 0;
/*
This method takes in a voltage and returns the resistance
*/
const int resistorValue = 2200; 
int calculateResistance( int voltage ){

  return ( (resistorValue*1023.) / voltage ) - resistorValue;
}
const int baseResistance[6] = {5353, 3316, 4877, 5373, 5373, 320 };

void updateLine( int resistance, int line ){
  int difference = baseResistance[line] - resistance;

   Serial.print( line );Serial.print(  ": "); Serial.print( resistance ); Serial.print( ", " ); Serial.println( difference );

  for( int i = 0; i < 4; i++ ){
    if( pieces[i][0] == line ){
      difference -= resistorValues[pieces[i][1]];
    }
  }

  int currentSpace = 0;
  bool spaceDetected = false;
  if( line == 5 ){
    //This is the mini line :)
    if( difference - resistorValues[0] <= 10 ){
      spaceDetected = true;
      currentSpace = 0;  
    }else if( difference - resistorValues[0] <= 10 ){
      spaceDetected = true;
      currentSpace = 1;
    }
  }else{
    for( int j = 0; j < 10; j++ ){
      if( difference - resistorValues[0] <= 10 ){
        spaceDetected = true;
        currentSpace = j; 
        break;
      } 
    }
  }

  if( spaceDetected ){
    pieces[currentTurn][0] = line;
    pieces[currentTurn][1] = currentSpace;
  }
}

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
Serial.println( "TEST" );
  for( int i = 0; i < sizeof(sensors); i++ ){
    pinMode(sensors[i], INPUT); 
  }
  laLloronaCoords[0] = (double)random( 25, 50 ) / 5.0;
  laLloronaCoords[1] = (double)random( 25, 50 ) / 5.0;
  //Code to begin game here

  //Code for first turns here

 // Init USB serial port for debugging
  // Init serial port for DFPlayer Mini
  softwareSerial.begin(9600);
 if (player.begin(softwareSerial)) {
   Serial.println("OK");

    // Set volume to maximum (0 to 30).
    player.volume(30);
    // Play the first MP3 file on the SD card
    player.play(1);
  } else {
    Serial.println("Connecting to DFPlayer Mini failed!");
  }
}

void loop() {
  
  //First we need to calculate our resistances to determine where the pieces are.
  //We will do this because we know where all but one piece is

  for( int i = 0; i < 6; i++ ){
    updateLine( calculateResistance( analogRead(i) ), i );
  }

  
  double minDistance = 100;

  for( int i = 0; i < 4; i++ ){
    if( pieces[i][0] == -1 ){
      double x = coords[pieces[i][0]][pieces[i][1]][0];
      double y = coords[pieces[i][0]][pieces[i][1]][1];

      distances[i] = sqrt( sq( x- laLloronaCoords[0]) + sq( y - laLloronaCoords[1]) );
      if( minDistance > distances[i] ){
        minDistance = distances[i];
      }
    }else{
      distances[i] = -1;
    }
  }
  
  // code for when - players move different amounts --> channge volume 
  // Set volume to maximum - when closest 
  player.volume(minDistance / 50);
    
    // Play the first MP3 file on the SD card - save mp3 file as 0001.mp3 for program to read it   
  //player.play(1);

  //Then, we need to calculate the distance to La Llorona.
  //Finally, we need to modify all of the values
  //We need checks here to see if anyone died :(
    
}




