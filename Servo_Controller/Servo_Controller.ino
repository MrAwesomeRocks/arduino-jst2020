// Import servo library
#include <Servo.h>

// Initialize Servo
Servo myServo;

// Set up potentiometer variables
int const potPin = A0;
int potVal;
int angle;

void setup(){
  myServo.attach(9); // Say servo attached to pin9
  Serial.begin(9600); // Start serial connection
}

void loop(){
  // Get raw poteniometer value and print it
  potVal = analogRead(potPin);
  Serial.print("potVal: ");
  Serial.print(potVal);

  // Get mapped poteniometer value and print it
  angle = map(potVal, 0, 1023, 0, 179);
  Serial.print(", angle: ");
  Serial.println(angle);

  // Output to server
  myServo.write(angle);
  delay(15);
}
