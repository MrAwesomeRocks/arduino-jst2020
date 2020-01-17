// Set up pins
const int enablePin = 11;
const int in1Pin = 10;
const int in2Pin = 9;
const int mOverride = 6;
const int mFor = 8;
const int mBack = 7;
const int potIn = A0;

void setup() {
  // initialize pins:
  pinMode(enablePin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(mOverride, INPUT);
  pinMode(mFor, INPUT);
  pinMode(mBack, INPUT);

  // start serial connection
  Serial.begin(9600);
  Serial.println("Enter s (stop), f, or r followed by Duty Cycle (0 to 255). E.g. f120");
}

void loop() {
  if (Serial.available()) { // check for input
    char direction = Serial.read();
    if (direction == 's'){
      stop();
      return;
    }
    int pwm = Serial.parseInt();
    if (direction == 'f'){
      goForward(pwm);
    }
    else if (direction == 'r'){
      goBackward(pwm);
    }
  }
  else if (digitalRead(mOverride) == HIGH){
    if (digitalRead(mFor) == HIGH){
      goForward(map(analogRead(potIn), 0, 1023, 0, 255));
    }
    else if (digitalRead(mBack) == HIGH){
      goBackward(map(analogRead(potIn), 0, 1023, 0, 255));
    }
    else{
      stop();
    }
  }
}

void goForward(int pwm) {
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  analogWrite(enablePin, pwm);
  Serial.print("Forward ");
  Serial.println(pwm);
}

void goBackward(int pwm) {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  analogWrite(enablePin, pwm);
  Serial.print("Reverse ");
  Serial.println(pwm);
}

void stop(){
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  analogWrite(enablePin, 0);
  Serial.print("Stop ");
}
