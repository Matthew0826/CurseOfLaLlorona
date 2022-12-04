// Rotary Encoder Inputs
 #define inputSW A3
 #define inputCLK A4
 #define inputDT A5
 #include <LiquidCrystal.h>

#define numbPieces 2

 #define moveConst 3
 
 const int ledPins[numbPieces] = { 6, 10 };
 int counter = 0; 
 int currentStateCLK;
 int previousStateCLK;
 double lastMillis = 0;

 int ids[numbPieces][2] = {{0, 0}, {0, 0}};
 int prevIds[numbPieces][2] = {{0, 0}, {0, 0}};
 bool modCol = true;
 
 String currentDir ="";
 unsigned long lastButtonPress = 0;
 LiquidCrystal lcd(A0, A1, 5, 4, 3, 2);

double laLlorona[2] = {0, 0};

 const int rowSize[6] = {10, 8, 7, 8, 4};
 const char rows[6] = {'A', 'B', 'C', 'D', 'E' };
 const char buttons[numbPieces] = {8, 9};
 bool alive[numbPieces] = {true, true};
const String buttonString[numbPieces] = {"Green", "Blue"};

int currentButton = 0;

const double coords[6][11][2] = {{{0.5, 1.5}, {0.5, 2.5}, {0.5, 3.5}, {0.5, 4.5}, {0.5, 5.5}, {0.5, 6.5}, {0.75, 7.5}, {1.75, 7.5}, {2.35, 6.2 }, {2.85, 5}, {3.85,5.3 } },
{{1.25, 1.25}, {2.25, 2.25}, {3.25, 3.25}, {3.75, 4}, {4.5, 4.5}, {5.5, 4.25}, {6.5, 4}, {7.5, 3.5}, {8.5, 3.25} },
{{1.75, 0.5}, {3, 0.5}, {4.5, 0.5}, {5.5, 0.75}, {6.5, 0.75}, {7.5, 0.75}, {8.5, 0.75}, {8.5, 2} },
{{2.75, 8}, {3.75, 8.25}, {4.65, 8.65}, {5.38, 9.25 }, {6.25, 8}, {7,7 }, {7.25, 6}, {6.37, 6.65}, {5.5, 5.25}  },
{{8.5, 4.5}, {8.5, 5.5}, {8.5, 6.5}, {8.75,7.25 },{9, 8.25}}};

 String encdir ="";
void updateLights(){
   for( int i = 0; i < numbPieces; i++ ){
     int a = laLlorona[0] - coords[ids[i][1]][ids[i][0]][0];
      int b = laLlorona[1] - coords[ids[i][1]][ids[i][0]][1];
      
      double distance = pow( a, 2) + pow( b, 2);
      Serial.print( i ); Serial.print( ": ");
      Serial.println( sqrt( distance ) );
      distance = sqrt( distance );
      int brightness = 0;
      if( distance <= 1.25 ){
        Serial.print( "PLAYER "); Serial.println( i );
        brightness = 0;
        alive[i] = false;
      }else if( distance <= 2.5 ){
        brightness = 255;
      }
      else if( distance <= 3 ){
        brightness = 100;
      }else if( distance <= 3.5 ){
        brightness = 25;
      }else if( distance <= 4 ){
        brightness = 15;
      }else{
        brightness = 5;
      }
      analogWrite(ledPins[i], brightness);
   }
 }
 void setup() { 
   laLlorona[0] = random(7, 8);
   laLlorona[1] = random(7, 8 );
   lcd.begin(16, 2);
  lcd.print("hello, world!");
   // Set encoder pins as inputs  
   pinMode (inputCLK,INPUT);
   pinMode (inputDT,INPUT);
   pinMode(inputSW, INPUT_PULLUP);
   pinMode( buttons[0], INPUT_PULLUP );
    pinMode( buttons[1], INPUT_PULLUP );

   // Set LED pins as outputs
   pinMode (ledPins[0],OUTPUT);
   pinMode (ledPins[1],OUTPUT);
   
   // Setup Serial Monitor
   Serial.begin(9600);
   // Read the initial state of inputCLK
   // Assign to previousStateCLK variable
   previousStateCLK = digitalRead(inputCLK);
  updateLights();
    randomSeed(millis());

 } 

 
 void moveLaLlorona(){
  Serial.println( "TEST" );
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

    int xMultiplier = 1;
    int yMultiplier = 1;

    int a = laLlorona[0] - coords[ids[i][1]][ids[i][0]][0];
    int b = laLlorona[1] - coords[ids[i][1]][ids[i][0]][1];

    double distance = pow( a, 2) + pow( b, 2);
    dx -= ( ( a * noise ) / distance );
    dy -= ( ( b * noise ) / distance );

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
        }
        if( currentButton = startingButton ){
          //Add some quirky everyone loses code :D
          break;
        }
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
         counter = 0;
         ids[currentButton][modCol]++;
       }
       encdir ="CCW";
       
     } else {
       // Encoder is rotating clockwise
       counter --;
       encdir ="CW";

        if( counter <= -moveConst ){
         counter = 0;
         ids[currentButton][modCol]--;
       }
       
     }
   } 
   // Update previousStateCLK with the current state
   previousStateCLK = currentStateCLK; 
   
  // Read the button state
  int btnState = digitalRead(inputSW);

  //If we detect LOW signal, button is pressed
    if (btnState == LOW) {
    //since 50ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 50) {
      modCol = !modCol;
    }


    // Remember last button press event
    lastButtonPress = millis();
  }
  // Put in a slight delay to help debounce the reading
  
  //This updates regularly & is our main code body
  if( ((double)millis() / 1000 ) - lastMillis > 0.25 ){
    checkCounter();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print( "Turn: " ); lcd.print( buttonString[currentButton]);
    lcd.setCursor(0, 1);
    lcd.print(rows[ids[currentButton][1]]);lcd.print(ids[currentButton][0] + 1);
    //lcd.print( "( " ); lcd.print( coords[ids[1]][ids[0]][0] );lcd.print( ", " ); 
    //lcd.print( coords[ids[1]][ids[0]][1] );lcd.print( " )" );
    lastMillis = (double)millis() / 1000;
  }

  delay(1);
}
