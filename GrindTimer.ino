// vi:ts=4

#include <Wire.h>
#include <hd44780.h>                        // main hd44780 header
#include <hd44780ioClass/hd44780_I2Cexp.h>  // i2c expander i/o class header
// #include <EEPROM.h>                      // EEPROM header for saving values on the board

hd44780_I2Cexp lcd;                         // declare lcd object

// LCD geometry
const int LCD_COLS = 16;
const int LCD_ROWS = 2;

// Button
const int buttonPin = 2;
int buttonState = 0;

// Relay
const int relayPin = 7;

// Potentiometer
int sensorPin = A3;
int sensorValue = 0;
int lastSensorValue = 0;
const int sensorMaxValue = 1015;

// Grind timer
float grindTimer = 20.0;
const float grindTimerMin = 5;            // Minimal grind timer setting in seconds
const float grindTimerMax = 60;           // Maximal grind timer setting in seconds

// Standby timer
const int standbyInterval = 30000;      // Enable standby in ms 
unsigned long previousTime = 0;
unsigned long currentTime = 0;

// Custom heart symbol hex
byte heart[] = {
  0x00,
  0x0A,
  0x1F,
  0x1F,
  0x0E,
  0x04,
  0x00,
  0x00
};

void setup() {
int status;

  status = lcd.begin(LCD_COLS, LCD_ROWS); // initialize display
  if(status)                              // non zero status means it was unsuccesful
  {
    hd44780::fatalError(status);          // does not return
  }

  lcd.createChar(1, heart);               // define custom heart icon
  
  showDefaultMsg();                       // print a default message to the LCD
 
  pinMode(buttonPin, INPUT);              // initialize the button pin as an input
  pinMode(relayPin, OUTPUT);              // initialize the relay pin as an output

  Serial.begin(115200);                   // initialize serial

  
  sensorValue = analogRead(sensorPin);    // Set initial value for sensor
  lastSensorValue = sensorValue;          // Set initial value for last read sensor value
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);
  sensorValue = analogRead(sensorPin);
  
  if (buttonState == LOW) {
    grindStart();
  }

  if ((sensorValue >= lastSensorValue + 2) || (sensorValue <= lastSensorValue - 2)) {
    Serial.print("sensor value: ");
    Serial.println(sensorValue);
    
    lastSensorValue = sensorValue;

    // Calculate timer's new value in miliseconds
    float factor = float(sensorValue) / float(sensorMaxValue);
    float newTimerValue = (grindTimerMax - grindTimerMin) * factor + grindTimerMin;
 
    setGrindTimer(newTimerValue);

    // Delay to avoid bouncing
    delay(100);
  }

  checkStandbyMode();       // check if standby mode needs to be enabled
}

//void grindStart() {
//
//  disableStandbyMode();
//
//  digitalWrite(relayPin, HIGH);
//  
//  for (float timer = float(grindTimer); timer >= 0.0; timer -= 0.1) {
//    
//    Serial.println(timer, 1);
//    lcd.clear();
//    lcd.print("Grinding...");
//    lcd.setCursor(0, 1);
//    lcd.print(timer, 1);
//    delay(100);
//  }
//
//  digitalWrite(relayPin, LOW);
//
//  grindCompleted();
//}

void grindStart() {

  disableStandbyMode();

  digitalWrite(relayPin, HIGH);

  unsigned long currentMillis = millis();
  unsigned long stopGrindAtMillis = currentMillis + (long(grindTimer) * 1000);

  Serial.println(currentMillis);
  Serial.println(stopGrindAtMillis);

  for (float timer = float(grindTimer); stopGrindAtMillis >= currentMillis; timer = float(stopGrindAtMillis - currentMillis) / 1000 ) {
    currentMillis = millis();
    Serial.println(timer);

    lcd.clear();
    lcd.print("Grinding...");
    lcd.setCursor(0, 1);
    lcd.print(timer, 1);

    delay(100);
  }

  digitalWrite(relayPin, LOW);

  grindCompleted();
}

void grindCompleted() {

  lcd.clear();
  lcd.print("Finished in:");
  lcd.setCursor(0, 1);
  lcd.print(grindTimer, 1);
  lcd.print("s");

  delay(2000);
  
  showDefaultMsg();
}

void setGrindTimer(float seconds) {
  disableStandbyMode();
  
  grindTimer = seconds;
  
  Serial.print("Timer set to: ");
  Serial.println(seconds);
  lcd.clear();
  lcd.print("Timer set to:");
  lcd.setCursor(0, 1);
  lcd.print(grindTimer, 1);
}

void showDefaultMsg() {
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print("I ");
  lcd.write(1);               // write data of custom heart symbol
  lcd.print(" Coffee");
}

void checkStandbyMode() {
  currentTime = millis();     // update current time with millis
  
  if (currentTime - previousTime >= standbyInterval) {
    lcd.noBacklight();

    previousTime = currentTime;
  }
}

void disableStandbyMode() {
  currentTime = millis();
  previousTime = currentTime;
  lcd.backlight();
}
