#include <AccelStepper.h>
#include "Adafruit_LiquidCrystal.h"

// ========================= PIN ASSIGNMENTS =========================

// DC motors (L293D)
const int motorL1 = 9;    // Left shuffle roller
const int motorL2 = 8;    
const int motorR1 = 7;    // Right shuffle roller
const int motorR2 = 6;
const int motorD1 = 5;    // Dealing roller (PWM)
const int motorD2 = 4;
const int motorS1 = 3;    // Shooting roller
const int motorS2 = 2;

// Stepper 28BYJ-48 + ULN2003
#define IN1 10
#define IN2 11
#define IN3 12
#define IN4 13

// Buttons
const int butPlayUp    = A2;
const int butPlayDown  = A3;
const int butStart     = A4;
const int butCardUp    = A5;
const int butCardDown  = A6;

// Photoresistors
const int phoLeft  = A1;
const int phoRight = A0;

// Random seed pin
const int randPin = A7;

// =============================== CONSTANTS ===============================

const unsigned long cardDelay = 300;
const int sensorThreshold = 1;

const int motorSpeedL = 255;
const int motorSpeedR = 255;
const int motorSpeedD = 255;
const int motorSpeedS = 255;

const unsigned long DEALING_FORWARD_TIME  = 1;
const unsigned long DEALING_BACKWARD_TIME = 150;

int numPlayers = 4;
int numCards   = 52;

// LCD
Adafruit_LiquidCrystal lcd(0);

// ========================= STEPPER CONFIG =========================

AccelStepper stepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

const float stepsPerRev = 2048.0;
const float stepsPerDegree = stepsPerRev / 360.0;

float currentAngle = 0.0;
int stepperSpeed = 1000;

// ========================= HELPER FUNCTIONS =========================

bool leftHasCards()  { return analogRead(phoLeft)  <= sensorThreshold; }
bool rightHasCards() { return analogRead(phoRight) <= sensorThreshold; }

void runMotor(int pin) {
  digitalWrite(pin, HIGH);
  delay(cardDelay);
  digitalWrite(pin, LOW);
}

// Dealing motor forward/back cycle (H-bridge)
void runMotorD_ForwardBack() {

  // Forward
  digitalWrite(motorD2, LOW);
  analogWrite(motorD1, motorSpeedD);
  delay(DEALING_FORWARD_TIME);
  digitalWrite(motorD1, LOW);

  // Backward
  digitalWrite(motorD2, HIGH);
  analogWrite(motorD1, motorSpeedD);
  delay(DEALING_BACKWARD_TIME);

  // Stop
  digitalWrite(motorD1, LOW);
  digitalWrite(motorD2, LOW);
}

void lcdShufflingAnimation() {
  lcd.clear();
  lcd.print("Shuffling...");
  for (int i = 0; i < 3; i++) {
    lcd.setCursor(0,1); lcd.print(".  ");  delay(250);
    lcd.setCursor(0,1); lcd.print(".. ");  delay(250);
    lcd.setCursor(0,1); lcd.print("...");  delay(250);
  }
}

void lcdDealingAnimation() {
  lcd.clear();
  lcd.print("Dealing cards");
  for (int i = 0; i < 5; i++) {
    lcd.setCursor(0,1); lcd.print(">");   delay(150);
    lcd.setCursor(0,1); lcd.print(">>");  delay(150);
    lcd.setCursor(0,1); lcd.print(">>>"); delay(150);
    lcd.setCursor(0,1); lcd.print("    "); delay(150);
  }
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Players: "); lcd.print(numPlayers);
  lcd.setCursor(0,1); lcd.print("Cards:   "); lcd.print(numCards);
}

// ========================= STEPPER CONTROL =========================

void setSpeed(int spd) {
  stepperSpeed = constrain(spd, 200, 1800);
  stepper.setMaxSpeed(stepperSpeed);
}

void rotateDegrees(float degrees) {
  float diff = degrees;

  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  long steps = diff * stepsPerDegree;
  stepper.move(steps);
  currentAngle += diff;

  if (currentAngle >= 360) currentAngle -= 360;
  if (currentAngle < 0)    currentAngle += 360;
}

// ========================= SERIAL TEST MODE =========================

void runTestMode() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("TEST MODE");
  lcd.setCursor(0,1); lcd.print("Serial Control");

  while (true) {

    stepper.run();

    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();

      if (cmd == "L1") digitalWrite(motorL1, HIGH);
      if (cmd == "L0") digitalWrite(motorL1, LOW);

      if (cmd == "R1") digitalWrite(motorR1, HIGH);
      if (cmd == "R0") digitalWrite(motorR1, LOW);

      if (cmd == "D1") digitalWrite(motorD1, HIGH);
      if (cmd == "D0") digitalWrite(motorD1, LOW);

      if (cmd == "DB") runMotorD_ForwardBack();

      if (cmd == "S1") digitalWrite(motorS1, HIGH);
      if (cmd == "S0") digitalWrite(motorS1, LOW);

      if (cmd == "CW")  rotateDegrees(90);
      if (cmd == "CCW") rotateDegrees(-90);

      if (cmd.startsWith("SPD")) {
        int sp = cmd.substring(3).toInt();
        setSpeed(sp);
      }

      if (cmd == "READ") {
        Serial.print("Left = ");  Serial.println(analogRead(phoLeft));
        Serial.print("Right = "); Serial.println(analogRead(phoRight));
      }
    }
  }
}

// ========================= SETUP =========================

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(randPin));

  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(motorD1, OUTPUT);
  pinMode(motorD2, OUTPUT);
  pinMode(motorS1, OUTPUT);
  pinMode(motorS2, OUTPUT);

  digitalWrite(motorL2, LOW);
  digitalWrite(motorR2, LOW);
  digitalWrite(motorD2, LOW);
  digitalWrite(motorS2, LOW);

  pinMode(butPlayUp,    INPUT_PULLUP);
  pinMode(butPlayDown,  INPUT_PULLUP);
  pinMode(butCardUp,    INPUT_PULLUP);
  pinMode(butCardDown,  INPUT_PULLUP);
  pinMode(butStart,     INPUT_PULLUP);

  lcd.begin(16,2);

  stepper.setMaxSpeed(stepperSpeed);
  stepper.setAcceleration(800);

  updateLCD();
}

// ========================= LOOP =========================

void loop() {

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "TEST") runTestMode();
  }

  if (!digitalRead(butPlayUp)) {
    if (numPlayers < 8) numPlayers++;
    updateLCD();
    delay(200);
  }

  if (!digitalRead(butPlayDown)) {
    if (numPlayers > 1) numPlayers--;
    updateLCD();
    delay(200);
  }

  if (!digitalRead(butCardUp)) {
    if (numCards < 100) numCards++;
    updateLCD();
    delay(200);
  }

  if (!digitalRead(butCardDown)) {
    if (numCards > 1) numCards--;
    updateLCD();
    delay(200);
  }

  if (!digitalRead(butStart)) {

    lcdShufflingAnimation();

    bool left  = leftHasCards();
    bool right = rightHasCards();

    while (left || right) {
      if (left && right) {
        if (random(0,2) == 0) runMotor(motorL1);
        else runMotor(motorR1);
      }
      else if (left)  runMotor(motorL1);
      else if (right) runMotor(motorR1);

      delay(55);
      left  = leftHasCards();
      right = rightHasCards();
    }

    lcdDealingAnimation();

    float angle = 360.0 / numPlayers;

    digitalWrite(motorS1, HIGH);

    int remaining = numCards;

    while (remaining > 0) {

      runMotorD_ForwardBack();

      rotateDegrees(angle);
      while (stepper.distanceToGo() != 0)
        stepper.run();

      remaining--;
    }

    digitalWrite(motorS1, LOW);

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("   Finished!");
    delay(1500);

    updateLCD();
  }

  stepper.run();
}
