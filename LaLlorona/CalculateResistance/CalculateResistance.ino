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

void setup() 
{ 
  // set up the sensor pin as an input and LED pin as an output  
  pinMode(sensorPin, INPUT); 
  pinMode(ledPin, OUTPUT); 
  Serial.begin(9600);
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
