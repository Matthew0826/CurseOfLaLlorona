// Rotary Encoder Inputs
 #define inputSW 3
 #define inputCLK 4
 #define inputDT 5
 
 // LED Outputs
 #define led1 9
 #define led2 10
 #define led3 11
 
 int counter = 0; 
 int currentStateCLK;
 int previousStateCLK;
 
 String currentDir ="";
 unsigned long lastButtonPress = 0;

 String encdir ="";
 
 void setup() { 
   
   // Set encoder pins as inputs  
   pinMode (inputCLK,INPUT);
   pinMode (inputDT,INPUT);
   pinMode(inputSW, INPUT_PULLUP);
   
   // Set LED pins as outputs
   pinMode (led1,OUTPUT);
   pinMode (led2,OUTPUT);
   pinMode (led3,OUTPUT);
   
   // Setup Serial Monitor
   Serial.begin (9600);
   
   // Read the initial state of inputCLK
   // Assign to previousStateCLK variable
   previousStateCLK = digitalRead(inputCLK);
 
 } 
 
 void loop() { 
  
  // Read the current state of inputCLK
   currentStateCLK = digitalRead(inputCLK);
    
   // If the previous and the current state of the inputCLK are different then a pulse has occured
   if (currentStateCLK != previousStateCLK){ 
       
     // If the inputDT state is different than the inputCLK state then 
     // the encoder is rotating counterclockwise
     if (digitalRead(inputDT) != currentStateCLK) { 
       counter --;
       encdir ="CCW";
       digitalWrite (led1, LOW);
       digitalWrite(led2, HIGH);
       
     } else {
       // Encoder is rotating clockwise
       counter ++;
       encdir ="CW";
       digitalWrite(led1, HIGH);
       digitalWrite(led2, LOW);
       
     }
     Serial.print("Direction: ");
     Serial.print(encdir);
     Serial.print(" -- Value: ");
     Serial.println(counter);
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
      digitalWrite(led3, HIGH);
    }

    if (btnState == HIGH) {
      digitalWrite(led3, LOW);
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

  // Put in a slight delay to help debounce the reading
  delay(1);
}
