//Include all the libraries required  for the LCD & for the Arduino
#include <LiquidCrystal.h>
#include "Arduino.h"
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"

// These are the inputs for the rotary encoder
 #define inputSW 2
 #define inputCLK 4
 #define inputDT 7

//Number of pieces in the game
#define numbPieces 4
//Number of clicks until the encoder changes the value
#define moveConst 3
 
static const uint8_t PIN_MP3_TX = 12; // Connects to module's RX 
static const uint8_t PIN_MP3_RX = 13; // Connects to module's TX 
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);      //Software Serial
DFRobotDFPlayerMini player;                                 //This is the actual mp3 player

//Player 1: Yellow
//Player 2: Red
//Player 3: Blue
//Player 4: Green
const int ledPins[numbPieces] = { 3, 5, 6, 9 };    //The LED pins for each player

//This is stuff for turning the knob 
int counter = 0;                                   //The counter for the encoder
int currentStateCLK;                               //The current state of the encoder
int previousStateCLK;                              //The previous state of the encoder
double lastMillis = 0;

//These variables are for the cards and for when a player dies
bool usedCard = false;           //Whether a revive card has been used
bool useAbility = false;         //Whether another card has been used
bool breakout = false;           //Wheter we need to display the revive prompt
int deadPlayer = 0;              //Which player is dead?

//This stores the IDs and the noises of each player
int ids[numbPieces][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};           //Where they are currently
int prevIds[numbPieces][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};       //Where they were last turn
double noises[numbPieces] = {0, 0, 0, 0};                            //How much noise they made

int modVal = 0;                            //This is the setting that we are at
String currentDir ="";                     //The current direction
unsigned long lastButtonPress = 0;         //The time that the last button was pressed
LiquidCrystal lcd(A0, A1, A2, A3, A4, A5); //The LCD display

//This is where La Llorona is 
double laLlorona[2] = {0, 0};

//This stores values for coordinates and buttons and such
const int rowSize[6] = {11, 8, 7, 8, 5};                                      //The size of each line
const char rows[6] = {'A', 'B', 'C', 'D', 'E' };                              //The name of each line
const char buttons[numbPieces] = {1, 8, 10, 11};                              //The button pins for each player
bool alive[numbPieces] = {false, false, false, false};                        //Whether each player is alive
const String buttonString[numbPieces] = {"Yellow", "Red", "Blue", "Green"};   //Name of each player

//Which is the current player
int currentButton = -1;

//These are all of the coordinates for the spaces. 
const double coords[6][12][2] = {{{0, 0}, {0.5, 1.5}, {0.5, 2.5}, {0.5, 3.5}, {0.5, 4.5}, {0.5, 5.5}, {0.5, 6.5}, {0.75, 7.5}, {1.75, 7.5}, {2.35, 6.2 }, {2.85, 5}, {3.85,5.3 } },
{{1.25, 1.25}, {2.25, 2.25}, {3.25, 3.25}, {3.75, 4}, {4.5, 4.5}, {5.5, 4.25}, {6.5, 4}, {7.5, 3.5}, {8.5, 3.25} },
{{1.75, 0.5}, {3, 0.5}, {4.5, 0.5}, {5.5, 0.75}, {6.5, 0.75}, {7.5, 0.75}, {8.5, 0.75}, {8.5, 2} },
{{2.75, 8}, {3.75, 8.25}, {4.65, 8.65}, {5.38, 9.25 }, {6.25, 8}, {7,7 }, {7.25, 6}, {6.37, 6.65}, {5.5, 5.25}  },
{{8.5, 4.5}, {8.5, 5.5}, {8.5, 6.5}, {8.75,7.25 },{9, 8.25}, {9.5, 9.5}}};

String encdir ="";          //Encoder direction

//This updates the lights and the speaker
void updateLights(){

  double minDistance = 1000;                  //This initializes the minimum distance to a really big value
  //Iterate through each piece
  for( int i = 0; i < numbPieces; i++ ){

    //If the piece is dead we don't care about it
    if( !alive[i] ){
      continue;             
    }

    //Figure out the directions of La Llorona
    int a = laLlorona[0] - coords[ids[i][1]][ids[i][0]][0];           //X difference
    int b = laLlorona[1] - coords[ids[i][1]][ids[i][0]][1];           //Y difference
      
    double distance = pow( a, 2 ) + pow( b, 2 );                      //The distance between la llorona and the player
    distance = sqrt( distance );                                      //Finish out the square root of the distance

    //If the current piece is the closest to la llorona
    if( distance < minDistance ){
      minDistance = distance;                                          
    }

    //This code block sets the brightness of the lights
    int brightness = 0;
    //If we are within the kill radius
    if( distance <= 0.5 ){
      lcd.clear();              
      lcd.print( "YOU DEAD" );
      deadPlayer = i;               //Set the dead player
      breakout = true;              //Set the breakout to true
      brightness = 0;               //Turn off the brightness
      alive[i] = false;             //Kill the player

    //This just modifies the brightness based on how close we are
    }else if( distance <= 3 ){
      brightness = 255;
    }else if( distance <= 4 ){
      brightness = 100;
    }else if( distance <= 5 ){
      brightness = 25;
    }else if( distance <= 6 ){
      brightness = 15;
    }else{
      brightness = 5;
    }

    //This just sets the brightness to the current LED    
    analogWrite(ledPins[i], brightness);
  }
  //Set the volume
  player.volume( (int)(( 2 / minDistance ) * 30) );
}

//This is the setup code for Arduino
void setup() { 
  //Start the mp3 player
  softwareSerial.begin(9600);           //Start the serial player
  player.begin(softwareSerial);         //Start the mp3 player
  player.stop();                        //Turon off the speaker

  //Initialize all of the pins
  pinMode( 0, INPUT );                  //This is our random pin
  randomSeed(analogRead(0));            //Lets make a random seed

  //Encoder pins initialization
  pinMode( inputCLK,INPUT );             
  pinMode( inputDT,INPUT );              
  pinMode(inputSW, INPUT_PULLUP );

  //Pin modes for all of the button
  pinMode( buttons[0], INPUT_PULLUP );
  pinMode( buttons[1], INPUT_PULLUP );
  pinMode( buttons[2], INPUT_PULLUP );
  pinMode( buttons[3], INPUT_PULLUP );
  
  //Output pins as output
  pinMode( ledPins[0], OUTPUT );
  pinMode( ledPins[1], OUTPUT );
  pinMode( ledPins[2], OUTPUT );
  pinMode( ledPins[3], OUTPUT );

  //Reset the LCDs and prompt for players 
  lcd.clear();
  lcd.print("Select Players");

  bool buttonPressed[4] = {false, false, false, false};     //Sets all initial button states to false

  //Wait until the button is pressed
  while( digitalRead( inputSW ) == HIGH || currentButton == -1 ){   
    currentButton = -1;                                             //Sets the current button to -1 
    for( int i = 0; i < 4; i++ ){
      if( digitalRead( buttons[i] ) == LOW && !buttonPressed[i] ){
        alive[i] = !alive[i];                                     //Toggle alive for the button
        buttonPressed[i] = true;                                  //Check if the button is pressed so we don't rapidly toggle
      }else if( digitalRead( buttons[i] ) == HIGH ){
        buttonPressed[i] = false;                                //This makes it so we can check again if the button is not pressed
      }

      if( alive[i] && currentButton == -1 ){
        currentButton = i;                                        //This is the starting button now. Yay!
      }
      digitalWrite( ledPins[i], alive[i] );                      //Turn on the light if it is alive
    }     
    delay( 10 );                                                   //Wait for a little bit
  }

  //Go through each pin
  for( int i = 0; i < 4; i++ ){
    digitalWrite( ledPins[i], 0 );                                //Reset each pin to off
  }   

  //Randomize where la llorona starts
  laLlorona[0] = random(3, 10);
  laLlorona[1] = random(3, 10 );
  lcd.begin(16, 2);           //Start the lcd

  //Turn on the speaker and loop it
  player.volume(5);
  player.loop(1);
   
  previousStateCLK = digitalRead(inputCLK);           //Set the previous state of the encoder
  updateLights();                                     //Update the lights and the speaker
} 

//Move La Llorona 
void moveLaLlorona(){
  double dx = 0;              //Movement in x direction
  double dy = 0;              //Movement in y direction

  //Go through each piece and check it
  for( int i = 0; i < numbPieces; i++ ){
    //If the piece is dead skip it
    if( !alive[i] ){
      continue;
    }

    //This dissapates the noise
    noises[i] *= 0.75;

    //This calculates noise by calculating distance moved
    double noise = pow( coords[ids[i][1]][ids[i][0]][0] - coords[prevIds[i][1]][prevIds[i][0]][0], 2) + 
    pow( coords[ids[i][1]][ids[i][0]][1] - coords[prevIds[i][1]][prevIds[i][0]][1], 2);
    noise = sqrt( noise );

    //Add movement noise to noise
    noises[i] += noise;
  
    //Calculate the x difference and y difference
    int a = laLlorona[0] - coords[ids[i][1]][ids[i][0]][0];
    int b = laLlorona[1] - coords[ids[i][1]][ids[i][0]][1];

    //Ensure la llorona moves in the right direction
    if( a > 0 ){
      dx += noises[i];
    }else if( a < 0 ){
      dx -= noises[i];
    }    
    if( b > 0 ){
      dy += noises[i];
    }else if( b < 0 ){
      dy -= noises[i];
    } 

    //Change the previous ids. We are done using them.
    prevIds[i][0] = ids[i][0]; prevIds[i][1] = ids[i][1];
  }
  double magnitude = sqrt( (dx * dx) + (dy * dy) );           //This is the magnitude of where she is moving
  
  //Move la llorona by a unit vector
  laLlorona[0] -= ( dx / magnitude ) * 0.75;
  laLlorona[1] -= ( dy / magnitude ) * 0.75;
}
 
//Check whether or not we need to reset
void checkCounter(){
  //If the row is too big to back to zero
  if( ids[currentButton][0] > rowSize[ids[currentButton][1]] ){
   ids[currentButton][0] = 0;
  }
  //If we are less than zero go to max
  if( ids[currentButton][0] < 0 ){
    ids[currentButton][0] = rowSize[ids[currentButton][1]];
  }
  //If the line selector is less than zero go to four
  if( ids[currentButton][1] < 0 ){
    ids[currentButton][1] = 4;
  }
  //If the line selector is less than four go to zero
  if( ids[currentButton][1] > 4 ){
    ids[currentButton][1] = 0;
  }  
}
 
//This repeats periodically
void loop() { 
  // Read the current state of inputCLK
  currentStateCLK = digitalRead(inputCLK);

  //If the current button is pressed :o
  if( digitalRead( buttons[currentButton]) == LOW ){
    lcd.clear();              //clear the lcd :)

    //If the player HASN'T moved ( bum bum bum )
    if( ids[currentButton][0] == prevIds[currentButton][0] && ids[currentButton][1] == prevIds[currentButton][1] ){
      noises[currentButton] += 3;                 //MAKE SOME NOISE!!!!!
    }

    //If someone has won because they are at the victory space 
    if( ids[currentButton][0] == 5 && ids[currentButton][1] == 4 ){
      lcd.setCursor( 0, 0 );          //Reset the cursor (zam)
      lcd.print( buttonString[currentButton] ); lcd.print( " escaped..." );       //Print that they escaped (blam)
      lcd.setCursor( 0, 1 );          //Go down a line (spam)
      lcd.print( "Rest o yall dead" );        //Print the next line (Pam (from the Office))
      player.stop();                  //Turn off the MP3

      //Turn off all of the quirky lights
      digitalWrite( ledPins[0], 0 );
      digitalWrite( ledPins[1], 0 );
      digitalWrite( ledPins[2], 0 );
      digitalWrite( ledPins[3], 0 );

      //Get outta heree
      exit(0);
    }

    int startingButton = currentButton;
    currentButton++;                        //Increase the current button
    if( currentButton >= numbPieces ){
      currentButton = 0;                    //If it's too big reset 
    }
      
    //This is gonna cause an infinite while loop but it's 1:30 AM :P
    //Lmao no it's not I fixed the problem
    while( !alive[currentButton] ){
      currentButton++;                     //Increment the current button  
      //If the current button is too big, reset it
      if( currentButton >= numbPieces ){
        currentButton = 0;
      }
      
      if( currentButton == startingButton ){
        //Everyone loses! Get outta the loop
        break;
      }
    }
    //SHE MOVES!!! ooooooo spooky
    moveLaLlorona();  
    //Update all the quirky things
    updateLights();
    
    prevIds[currentButton][0] = ids[currentButton][0];          //Reset the previous ids
    prevIds[currentButton][1] = ids[currentButton][1];          //Reset the previous ids
  }

  // If the previous and the current state of the inputCLK are different then a pulse has occured
  if (currentStateCLK != previousStateCLK){ 
  // If the inputDT state is different than the inputCLK state then the encoder is rotating counterclockwise
  if (digitalRead(inputDT) != currentStateCLK) { 
    counter ++;               //Increment the counter
    lcd.clear();              //Clear the lcd

    //If counter is bigger than the move constant  
    if( counter >= moveConst ){
      //If we are in breakout mode, change whether we are using a card
      if( breakout ){
        usedCard = !usedCard;
      //Otherwise
      }else{
        //If the modified value is less than two we are not in cards mode
        if( modVal < 2 ){
          ids[currentButton][modVal]++; 
        }else{
          useAbility = !useAbility;         //Alternate ability mode
        }
      }
      counter = 0;            //Reset counter
    }
    encdir ="CCW";

    } else {
      // Encoder is rotating clockwise
      lcd.clear();          //Clear the lcd
      counter --;           //Decrement counter
      encdir ="CW";         

      //If we have gone far down enough
      if( counter <= -moveConst ){
        //If we are in breakout mode
        if( breakout ){
          usedCard = !usedCard;         //Toggle usedCard
        }else{
          //If we are not in card mode
          if( modVal < 2 ){
            ids[currentButton][modVal]--; 
          }else{
            useAbility = !useAbility;       //Otherwise toggle whether we are using the card
          }
        }
        counter = 0;              //Reset the counter
      }
    }
  } 
  // Update previousStateCLK with the current state
  previousStateCLK = currentStateCLK; 
   
  // Read the button state
  int btnState = digitalRead(inputSW);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //If we are in breakout mode
    if( breakout ){
      //If the card is used, bring the player back
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
      lcd.clear();        //clear the lcd
      //If we are in noise mode
      if( modVal == 3 && useAbility ){
        //Make some noise
        noises[currentButton] += 6;
        useAbility = false;
      }
      modVal++;         //Increment the modVal 
      //Reset the modVal
      if( modVal > 3 ){
        modVal = 0;
      }
    }
    // Remember last button press event
    lastButtonPress = millis();
  }
  
  //This updates regularly
  if( ((double)millis() / 1000 ) - lastMillis > 0.05 ){
    //If we are in breakout mode
    if( breakout ){
      //Reset cursor
      lcd.setCursor(0, 0);
      lcd.print( buttonString[deadPlayer] );      //Print who is dead        
      lcd.print( " is Dead!" );
      lcd.setCursor( 0, 1 );
      lcd.print( "Used Card? " );
      //Print whether they used card
      if( usedCard ){
        lcd.print( "Y" );
      }else{
        lcd.print( "N" );
      }
    //Otherwise
    }else{
      checkCounter();             //Check whether or not we need to reset
      lcd.clear(); //Reset the lcd
      lcd.setCursor(0, 0);          //Set the cursor to (0, 0)
      lcd.print( "Turn: " ); lcd.print( buttonString[currentButton]);       //Print whose turn
      lcd.setCursor(0, 1);
      //If we are in modVal mode
      if( modVal < 2 ){
        lcd.print(rows[ids[currentButton][1]]);lcd.print(ids[currentButton][0] + 1);   
      //We are in Switch mode
      }else if( modVal == 2 ){
        lcd.print( "Switch? " );
        if( useAbility ){
          lcd.print( "Y" );
          for( int i = 0; i < 4; i++ ){
            //If a button is pressed
            if( digitalRead( buttons[i] ) == LOW ){
              int tempId0 = ids[currentButton][0];     int tempId1 = ids[currentButton][1];       //Set a temp ID
              ids[currentButton][0] = ids[i][0];  ids[currentButton][1] = ids[i][1];              //Do a swappy swap
              ids[i][0] = tempId0;        ids[i][1] = tempId1;                                    //Finalize the swappy swap

              //Set the previous IDs so this doesn't make a lot of noise
              prevIds[i][0] = ids[i][0];   prevIds[i][1] = ids[i][1];
              prevIds[currentButton][0] = ids[i][0];   prevIds[currentButton][1] = ids[i][1];     
              useAbility = false;
              modVal = 0;    
            }
          }
        }else{
          lcd.print( "N" );
        }
      //This is just the make noise thing
      }else{
        lcd.print( "Make noise? " );
        if( useAbility ){
          lcd.print( "Y" );
        }else{
          lcd.print( "N" );
        }
      }
      //Remember the last press
      lastMillis = (double)millis() / 1000;
    }
  }
  //Wait for a little bit
  delay(1);
}
