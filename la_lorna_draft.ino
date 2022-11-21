//setting up mp3 player and speaker 
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
SoftwareSerial mySoftwareSerial(10, 11); // RX, TX - pins 
// Creating the speaker object
DFRobotDFPlayerMini speaker;

//setting up leds, buttons, & sensors for each player 
//const int sensor0 = A0;
const int sensor1 = A1;
const int sensor2 = A2;
const int sensor3 = A3;
//const int button0 = 0;
const int button1 = 2;
const int button2 = 4;
//const int button3 = 3;
//const int led0 = 7;
const int led1 = 5;
const int led2 = 6;
//const int led3 = 9;



void setup() {
  // put your setup code here, to run once:
 // pinMode(button0, INPUT_PULLUP);
pinMode(button1, INPUT_PULLUP);
pinMode(button2, INPUT_PULLUP);
//pinMode(button3, INPUT_PULLUP);
//pinMode(led0, OUTPUT);
pinMode(led1, OUTPUT);
pinMode(led2, OUTPUT);
//pinMode(led3, OUTPUT);

}

void loop() {
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
  if (sensor1 == HIGH){
    digitalWrite(led1, HIGH);
  }
  else {
    digitalWrite(led1, LOW);
  }
}
if (buttonState2 == HIGH){
  if (sensor2 == HIGH){
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
*/
}
