// Define digital pins
const int greenLEDPin = 9;
const int redLEDPin = 11;
const int blueLEDPin = 10;

// Define analog Pins
const int redSensorPin = A0;
const int greenSensorPin = A1;
const int blueSensorPin = A2;

// Initialize Variables
int redValue = 0;
int greenValue = 0;
int blueValue = 0;

int redSensorValue = 0;
int greenSensorValue = 0;
int blueSensorValue = 0;

// Make high and low variables for mapping
int redHigh = 0;
int greenHigh = 0;
int blueHigh = 0;

int redLow = 1023;
int greenLow = 1023;
int blueLow = 1023;

void setup(){
  Serial.begin(9600); // Start serial output

  // Initialize pins
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);// For configuration
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  
  // Get baseline highs
  digitalWrite(LED_BUILTIN, HIGH); // Tell user that it is time to configure highs

  while (millis() < 6000){  // Since paused for 1000 earlier, wait until millis = 6000
    redSensorValue = analogRead(redSensorPin);
    greenSensorValue = analogRead(greenSensorPin);
    blueSensorValue = analogRead(blueSensorPin);

    if (redSensorValue > redHigh){
      redHigh = redSensorValue;
    }
    if (greenSensorValue > greenHigh){
      greenHigh = greenSensorValue;
    }
    if (blueSensorValue > blueHigh){
      blueHigh = blueSensorValue;
    }
  }

  // Blink led to tell user to cover board
  digitalWrite(LED_BUILTIN, LOW);
  delay(250); //6250
  digitalWrite(LED_BUILTIN, HIGH);
  delay(250); //6500
  digitalWrite(LED_BUILTIN, LOW);
  delay(500); //7000
  digitalWrite(LED_BUILTIN, HIGH);

  // Get baseline lows
  while (millis() < 12000){
    redSensorValue = analogRead(redSensorPin);
    greenSensorValue = analogRead(greenSensorPin);
    blueSensorValue = analogRead(blueSensorPin);
    
    if (redSensorValue < redLow){
      redLow = redSensorValue;
    }
    if (greenSensorValue < greenLow){
      greenLow = greenSensorValue;
    }
    if (blueSensorValue < blueLow){
      blueLow = blueSensorValue;
    }
  }
}

void loop(){
  // Read value of photoresistors
  redSensorValue = analogRead(redSensorPin);
  delay(5);
  greenSensorValue = analogRead(greenSensorPin);
  delay(5);
  blueSensorValue = analogRead(blueSensorPin);

  // Map the sensor values for PWM Pins
  redValue = map(redSensorValue, redLow, redHigh, 5, 250);
  greenValue = map(greenSensorValue, greenLow, greenHigh, 5, 250);
  blueValue = map(blueSensorValue, blueLow, blueHigh, 5, 250);

  // Print raw sensor values
  Serial.print("Raw Sensor Values \t Red: ");
  Serial.print(redSensorValue);
  Serial.print("\t Green: ");
  Serial.print(greenSensorValue);
  Serial.print("\t Blue: ");
  Serial.print(blueSensorValue);
  // Print mapped sensor values on same line, but off to the side
  Serial.print("\t \t Mapped Sensor Values \t Red: ");
  Serial.print(redValue);
  Serial.print("\t Green: ");
  Serial.print(greenValue);
  Serial.print("\t Blue: ");
  Serial.println(blueValue);

  // Using PWM, modulate the LED brightness depending on the photoresistor brightness
  analogWrite(redLEDPin, redValue);
  analogWrite(greenLEDPin, greenValue);
  analogWrite(blueLEDPin, blueValue);
}
