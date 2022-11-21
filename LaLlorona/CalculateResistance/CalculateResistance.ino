/* 
 Nightlight 
 This program illuminates an LED, making it brighter as the room is dimmer 
*/ 
 
 
// give names to the pins we use and create variables for room and LED brightness 
const int sensorPin = A1; 
const int ledPin = 9; 
 double roomBrightness; 
 int ledBrightness; 
 int resistance;
int number = 0;
double sum = 0;
const int resistorValues[8] = {100, 220, 330, 470, 510, 680, 1010, 2000};
const int pieceValues[4] = {0, 22, 100, 220 };

//setting up mp3 player and speaker 
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
SoftwareSerial mySoftwareSerial(10, 11); // RX, TX - pins 
// Creating the speaker object
DFRobotDFPlayerMini speaker;

//setting up leds, buttons, & sensors for each player 
//const int sensor0 = A0;
//const int sensor1 = A1;
//const int sensor2 = A2;
//const int sensor3 = A3;
//const int button0 = 0;
const int button1 = 2;
const int button2 = 4;
//const int button3 = 3;
//const int led0 = 7;
const int led1 = 5;
const int led2 = 6;
//const int led3 = 9;

void setup() 
{ 
  // set up the sensor pin as an input and LED pin as an output  
  pinMode(sensorPin, INPUT); 
  pinMode(ledPin, OUTPUT); 
  Serial.begin(9600);

    // put your setup code here, to run once:
 // pinMode(button0, INPUT_PULLUP);
pinMode(button1, INPUT_PULLUP);
pinMode(button2, INPUT_PULLUP);
//pinMode(button3, INPUT_PULLUP);
//pinMode(led0, OUTPUT);
pinMode(led1, OUTPUT);
pinMode(led2, OUTPUT);
//pinMode(led3, OUTPUT);

// put your main code here, to run repeatedly:
  //fix the resistance 
 // buttonState0 = digitalRead(button0);
int buttonState1 = digitalRead(button1);
int buttonState2 = digitalRead(button2);
//buttonState3 = digitalRead(button3);
//if (buttonState0 = LOW){
  //digitalWrite(led0, LOW);
//}
if(buttonState1 == LOW){
  digitalWrite(led1, LOW);
}
if (buttonState2 == LOW){
  digitalWrite(led2, LOW);
}
//if else (buttonState3 = LOW){
  //digitalWrite(led3, LOW);
//}
/*if (buttonState0 = HIGH){
  if (sensor0 = HIGH){
    digitalWrite(led0, HIGH);
  }
  else {
    digitalWrite(led0, LOW);
  }
}
*/
if (buttonState1 == HIGH){
  if (sensor1 == HIGH){ // add resistor part 
    digitalWrite(led1, HIGH);
  }
  else {
    digitalWrite(led1, LOW);
  }
}
if (buttonState2 == HIGH){
  if (sensor2 == HIGH){ //add resistor part 
    digitalWrite(led2, HIGH);
  }
  else {
    digitalWrite(led2, LOW);
  }
}
/*if else (buttonState3 = HIGH){
  if (sensor3 = HIGH){
    digitalWrite(led3, HIGH);
  }
  else {
    digitalWrite(led3, LOW);
  }
}
}

int* calculateDifferences( double resistance ){
  double difference = 5373 - resistance;
  static int x[2];
  Serial.println(difference  );


  for( int i = 0; i < 8; i++ ){
    for( int j = 0; j < 4; j++ ){
      double currResistance;
      if( pieceValues[j] == 0 ){
        currResistance = 0;

      }else{
        currResistance = 1 / ((1/(double)resistorValues[i]) + (1/(double)pieceValues[j]));
      }
 
      double diffy = (resistorValues[i] - currResistance ) - difference;
      if( diffy <= 10 && diffy >= -10 ){
        x[0] = i;
        x[1] = j;

        return x;
      }
    }
  }
  x[0] = -1;
  x[1] = -1;
  return x;
}
 
 
void loop() 
{ 
  // measure the voltage coming from the photoresistor 
  // This number can range between 0 (GND) and 1023 (5V) 
  sum = 0;
  for( int i = 0; i < 300; i++ ){
    int currReading = analogRead(sensorPin);
    sum += (double)currReading / 100;
    delay(5);
  }

  roomBrightness = (sum / 300) * 100;

  // Mathematically change roomBrightness, which ranges in theory from 0->1023, 
  // but in practice from about 100->900, into a value to send to analogWrite. 
  // Remember analogWrite uses values from 255->0 (note the reversed range) 
  ledBrightness = 255 - ((roomBrightness - 100) / 3.5); 
  resistance = ( (2000*1025.) / roomBrightness ) - 2000;
  Serial.println( resistance );
  int* ptr = calculateDifferences( resistance );
  Serial.print( "Piece: " );
  Serial.println( ptr[1] );
  Serial.print( "Space: " );
  Serial.println( ptr[0] );


  // output this new brightness level to the LED 
  analogWrite(ledPin, ledBrightness) ; 
  delay( 100 );
} 
