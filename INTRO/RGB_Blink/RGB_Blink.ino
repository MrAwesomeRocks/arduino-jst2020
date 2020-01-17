// define pins:
int bluePin = 11;
int greenPin = 10;
int redPin = 9;

void setup() {
  // initialize pins
  pinMode(bluePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(redPin, OUTPUT);

  // turn on all pins, because using RGB LED
  digitalWrite(bluePin, HIGH);
  digitalWrite(greenPin, HIGH);
  digitalWrite(redPin, HIGH);
}

void loop() {
  digitalWrite(bluePin, HIGH); // turn off blue LED
  digitalWrite(redPin, LOW);   // turn on Red LED
  delay(1000);                 // wait one second
  digitalWrite(redPin, HIGH);  // red LED off
  digitalWrite(greenPin, LOW); // green LED on
  delay(1000);                 // wait one second
  digitalWrite(greenPin, HIGH);// green LED off
  digitalWrite(bluePin, LOW);  // blue LED on
  delay(1000);
}
