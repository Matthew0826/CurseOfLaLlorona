const int sensor0 = A0;
const int sensor1 = A1;
const int sensor2 = A2;
const int sensor3 = A3;
const int button0 = 0;
const int button1 = 1;
const int button2 = 2;
const int led0 = 3;
const int button3 = 4;
const int led1 = 5;
const int led2 = 6;
const int led3 = 9;

void setup() {
pinMode(button0, INPUT_PULLUP);
pinMode(button1, INPUT_PULLUP);
pinMode(button2, INPUT_PULLUP);
pinMode(button3, INPUT_PULLUP);
pinMode(led0, OUTPUT);
pinMode(led1, OUTPUT);
pinMode(led2, OUTPUT);
pinMode(led3, OUTPUT);
}

void loop() {
buttonState0 = digitalRead(button0);
buttonState1 = digitalRead(button1);
buttonState2 = digitalRead(button2);
buttonState3 = digitalRead(button3);
if (buttonState0 = LOW){
  digitalWrite(led0, LOW);
}
if else (buttonState1 = LOW){
  digitalWrite(led1, LOW);
}
if else (buttonState2 = LOW){
  digitalWrite(led2, LOW);
}
if else (buttonState3 = LOW){
  digitalWrite(led3, LOW);
}
if (buttonState0 = HIGH){
  if (sensor0 = HIGH){
    digitalWrite(led0, HIGH);
  }
  else {
    digitalWrite(led0, LOW);
  }
}
if else (buttonState1 = HIGH){
  if (sensor1 = HIGH){
    digitalWrite(led1, HIGH);
  }
  else {
    digitalWrite(led1, LOW);
  }
}
if else (buttonState2 = HIGH){
  if (sensor2 = HIGH){
    digitalWrite(led2, HIGH);
  }
  else {
    digitalWrite(led2, LOW);
  }
}
if else (buttonState3 = HIGH){
  if (sensor3 = HIGH){
    digitalWrite(led3, HIGH);
  }
  else {
    digitalWrite(led3, LOW);
  }
}
}
