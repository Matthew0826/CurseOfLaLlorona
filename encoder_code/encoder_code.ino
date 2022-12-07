//Include all the libraries required  for the LCD & for the Arduino
#include <LiquidCrystal.h>
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// These are the inputs for the rotary encoder
 #define inputSW 2
 #define inputCLK 4
 #define inputDT 7

#define numbPieces 4

 #define moveConst 3
 
 // Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = 12; // Connects to module's RX 
static const uint8_t PIN_MP3_RX = 13; // Connects to module's TX 
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);
// Create the Player object
DFRobotDFPlayerMini player;

 //Player 1: Yellow- 3, 10
 //Player 2: Red- 5
  //Player 3: Blue
  //Player 4: Green
 const int ledPins[numbPieces] = { 3, 5, 6, 9 };
 int counter = 0; 
 int currentStateCLK;
 int previousStateCLK;
 double lastMillis = 0;

 bool usedCard = false;
 bool breakout = false;
 int deadPlayer = 0;

 int ids[numbPieces][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
 int prevIds[numbPieces][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
 bool modCol = true;
 
 String currentDir ="";
 unsigned long lastButtonPress = 0;
 LiquidCrystal lcd(A0, A1, A2, A3, A4, A5);

double laLlorona[2] = {0, 0};

 const int rowSize[6] = {11, 8, 7, 8, 5};
 const char rows[6] = {'A', 'B', 'C', 'D', 'E' };
 const char buttons[numbPieces] = {1, 8, 10, 11};
 bool alive[numbPieces] = {false, false, false, false};
const String buttonString[numbPieces] = {"Yellow", "Red", "Blue", "Green"};

int currentButton = -1;

SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;

//These are all of the coordinates for the spaces. 
const double coords[6][12][2] = {{{0, 0}, {0.5, 1.5}, {0.5, 2.5}, {0.5, 3.5}, {0.5, 4.5}, {0.5, 5.5}, {0.5, 6.5}, {0.75, 7.5}, {1.75, 7.5}, {2.35, 6.2 }, {2.85, 5}, {3.85,5.3 } },
{{1.25, 1.25}, {2.25, 2.25}, {3.25, 3.25}, {3.75, 4}, {4.5, 4.5}, {5.5, 4.25}, {6.5, 4}, {7.5, 3.5}, {8.5, 3.25} },
{{1.75, 0.5}, {3, 0.5}, {4.5, 0.5}, {5.5, 0.75}, {6.5, 0.75}, {7.5, 0.75}, {8.5, 0.75}, {8.5, 2} },
{{2.75, 8}, {3.75, 8.25}, {4.65, 8.65}, {5.38, 9.25 }, {6.25, 8}, {7,7 }, {7.25, 6}, {6.37, 6.65}, {5.5, 5.25}  },
{{8.5, 4.5}, {8.5, 5.5}, {8.5, 6.5}, {8.75,7.25 },{9, 8.25}, {9.5, 9.5}}};

 String encdir ="";
void updateLights(){
  
  double minDistance = 1000;
   for( int i = 0; i < numbPieces; i++ ){
     if( !alive[i] ){
       continue;
     }
     int a = laLlorona[0] - coords[ids[i][1]][ids[i][0]][0];
      int b = laLlorona[1] - coords[ids[i][1]][ids[i][0]][1];
      
      double distance = pow( a, 2) + pow( b, 2);
      distance = sqrt( distance );
      if( distance < minDistance ){
        minDistance = distance;
      }
      int brightness = 0;
      if( distance <= 1.25 ){
        lcd.clear();
        lcd.print( "YOU DEAD" );
        deadPlayer = i;
        breakout = true;
        brightness = 0;
        alive[i] = false;
      }else if( distance <= 3 ){
        brightness = 255;
      }
      else if( distance <= 4 ){
        brightness = 100;
      }else if( distance <= 5 ){
        brightness = 25;
      }else if( distance <= 6 ){
        brightness = 15;
      }else{
        brightness = 5;
      }
      analogWrite(ledPins[i], brightness);
   }
   player.volume( (int)(( 2 / minDistance ) * 30) );
 }
 void setup() { 
   // Setup Serial Monitor
   Serial.begin(9600);
     softwareSerial.begin(9600);
// Start communication with DFPlayer Mini
  player.begin(softwareSerial);
  player.stop();
  
   pinMode (inputCLK,INPUT);
   pinMode (inputDT,INPUT);
   pinMode(inputSW, INPUT_PULLUP);
   pinMode( buttons[0], INPUT_PULLUP );
   // Set LED pins as outputs
   pinMode (ledPins[0],OUTPUT);
   pinMode (ledPins[1],OUTPUT);
  pinMode (ledPins[2],OUTPUT);

   pinMode (ledPins[3],OUTPUT);
    pinMode( buttons[1], INPUT_PULLUP );
    pinMode( buttons[2], INPUT_PULLUP );
    pinMode( buttons[3], INPUT_PULLUP );
    lcd.clear();
    lcd.print("Select Players");
    //lcd.print("Who will be playing?");

    bool buttonPressed[4] = {false, false, false, false};
   while( digitalRead( inputSW ) == HIGH || currentButton == -1 ){
     currentButton = -1;
     for( int i = 0; i < 4; i++ ){
       if( digitalRead( buttons[i] ) == LOW && !buttonPressed[i] ){
         alive[i] = !alive[i];
          buttonPressed[i] = true;
       }else if( digitalRead( buttons[i] ) == HIGH ){
         buttonPressed[i] = false;
       }

       if( alive[i] && currentButton == -1 ){
         currentButton = i;
       }
        digitalWrite( ledPins[i], alive[i] );
     }     
     delay( 10 );
   }

   for( int i = 0; i < 4; i++ ){
        digitalWrite( ledPins[i], 0 );
     }   


   laLlorona[0] = random(7, 8);
   laLlorona[1] = random(7, 8 );
   lcd.begin(16, 2);
   // Set encoder pins as inputs  
   

   

     player.volume(5);

    player.loop(1);
   
/*
    mySoftwareSerial.begin(9600);
    if( !myDFPlayer.begin(mySoftwareSerial) ){
      Serial.println( "DF PLAYER FAILED" );
    }else{
      Serial.println( "DF PLAYER OK" );      
    }
    */
    //myDFPlayer.volume(15);  //Set volume value. From 0 to 30
    //myDFPlayer.play(1);  //Play the first mp3
   // Read the initial state of inputCLK
   // Assign to previousStateCLK variable
   previousStateCLK = digitalRead(inputCLK);
  updateLights();
    randomSeed(millis());

 } 

 
 void moveLaLlorona(){
  double dx = 0;
  double dy = 0;
  for( int i = 0; i < numbPieces; i++ ){
    if( !alive[i] ){
      continue;
    }
    //Serial.print( i ); Serial.print( ": ");Serial.print(coords[ids[i][1]][ids[i][0]][0] );
    //Serial.print( ", " ); Serial.println( coords[ids[i][1]][ids[i][0]][1] );
    double noise = pow( coords[ids[i][1]][ids[i][0]][0] - coords[prevIds[i][1]][prevIds[i][0]][0], 2) + 
    pow( coords[ids[i][1]][ids[i][0]][1] - coords[prevIds[i][1]][prevIds[i][0]][1], 2);
    noise = sqrt( noise );


    int a = laLlorona[0] - coords[ids[i][1]][ids[i][0]][0];
    int b = laLlorona[1] - coords[ids[i][1]][ids[i][0]][1];

    double distance = pow( a, 2) + pow( b, 2);
    dx -= ( ( a * noise ) / distance ) * 1.5;
    dy -= ( ( b * noise ) / distance ) * 1.5;

  }
  if( dx >= 1.5 ){
    dx = 1.5;
  }else if( dx <= 1.5 ){
    dx = -1.5;
  }

  if( dy >= 1.5 ){
    dy = 1.5;
  }else if( dy <= 1.5 ){
    dy = -1.5;
  }
  laLlorona[0] += dx;
  laLlorona[1] += dy;

  Serial.print( laLlorona[0] ); Serial.print( ", " ); Serial.println( laLlorona[1] );
   
   //lcd.print( laLlorona[0] ); lcd.print( ", " ); lcd.print( laLlorona[1] );
 }
 

 void checkCounter(){
   if( ids[currentButton][0] > rowSize[ids[currentButton][1]] ){
     ids[currentButton][0] = 0;
   }
   if( ids[currentButton][0] < 0 ){
     ids[currentButton][0] = rowSize[ids[currentButton][1]];
   }
   if( ids[currentButton][1] < 0 ){
     ids[currentButton][1] = 4;
   }
   if( ids[currentButton][1] > 4 ){
     ids[currentButton][1] = 0;
   }  
 }
 
 void loop() { 
  // Read the current state of inputCLK
   currentStateCLK = digitalRead(inputCLK);
    if( digitalRead( buttons[currentButton]) == LOW ){
      lcd.clear();
      Serial.print(coords[ids[currentButton][1]][ids[currentButton][0]][0]  ); Serial.print( ", " ); Serial.println(coords[ids[currentButton][1]][ids[currentButton][0]][1]);
      if( ids[currentButton][0] == 5 && ids[currentButton][1] == 4 ){
        lcd.setCursor( 0, 0 );
        lcd.print( buttonString[currentButton] ); lcd.print( " escaped..." );
        lcd.setCursor( 0, 1 );
        lcd.print( "Rest o yall dead" );
        player.stop();
        digitalWrite( ledPins[0], 0 );
                digitalWrite( ledPins[1], 0 );

        digitalWrite( ledPins[2], 0 );

        digitalWrite( ledPins[3], 0 );
        exit(0);
      }
      int startingButton = currentButton;
      currentButton++;
      if( currentButton >= numbPieces ){
        moveLaLlorona();
        currentButton = 0;
      }
      
      //This is gonna cause an infinite while loop but it's 1:30 AM :P
      while( !alive[currentButton] ){
        currentButton++;
        if( currentButton >= numbPieces ){
          currentButton = 0;
          moveLaLlorona();
        }
        if( currentButton == startingButton ){
          //Add some quirky everyone loses code :D
          break;
        }
      }
      if( startingButton < currentButton ){

      }
      updateLights();
      prevIds[currentButton][0] = ids[currentButton][0];
      prevIds[currentButton][1] = ids[currentButton][1];
    }
   // If the previous and the current state of the inputCLK are different then a pulse has occured
   if (currentStateCLK != previousStateCLK){ 
     // If the inputDT state is different than the inputCLK state then 
     // the encoder is rotating counterclockwise
     if (digitalRead(inputDT) != currentStateCLK) { 
       counter ++;
       if( counter >= moveConst ){
         if( breakout ){
          usedCard = !usedCard;
         }else{
          ids[currentButton][modCol]++;           
         }
        counter = 0;

       }

       encdir ="CCW";
       
     } else {
       // Encoder is rotating clockwise
       counter --;
       encdir ="CW";

        if( counter <= -moveConst ){
          if( breakout ){
            usedCard = !usedCard;
          }else{
            ids[currentButton][modCol]--;
          }
          counter = 0;

         
       }
       
     }
   } 
   // Update previousStateCLK with the current state
   previousStateCLK = currentStateCLK; 
   
  // Read the button state
  int btnState = digitalRead(inputSW);

  //If we detect LOW signal, button is pressed
    if (btnState == LOW) {
      if( breakout ){
        if( usedCard ){
          alive[deadPlayer] = true;
          ids[deadPlayer][0] = 0; ids[deadPlayer][1] = 0;
          prevIds[deadPlayer][0] = 0; prevIds[deadPlayer][1] = 0;
          updateLights();
          player.stop();
        }
       breakout = false;
       lcd.clear();
     }
    //since 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    else if (millis() - lastButtonPress > 50) {
      modCol = !modCol;
    }


    // Remember last button press event
    lastButtonPress = millis();
  }
  // Put in a slight delay to help debounce the reading
  
  //This updates regularly & is our main code body
  if( ((double)millis() / 1000 ) - lastMillis > 0.05 ){
    if( breakout ){
        lcd.setCursor(0, 0);

        lcd.print( buttonString[deadPlayer] );        
        lcd.print( " is Dead!" );
        lcd.setCursor( 0, 1 );
        lcd.print( "Used Card? " );
        if( usedCard ){
          lcd.print( "Y" );
        }else{
          lcd.print( "N" );
        }
    }else{
    checkCounter();
    //lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print( "Turn: " ); lcd.print( buttonString[currentButton]);
    lcd.setCursor(0, 1);
    lcd.print(rows[ids[currentButton][1]]);lcd.print(ids[currentButton][0] + 1);
    //lcd.print( "( " ); lcd.print( coords[ids[1]][ids[0]][0] );lcd.print( ", " ); 
    //lcd.print( coords[ids[1]][ids[0]][1] );lcd.print( " )" );
    lastMillis = (double)millis() / 1000;
    }
    
  }

  delay(1);
}
