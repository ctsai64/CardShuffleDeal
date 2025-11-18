#include <AccelStepper.h>
#include "Adafruit_LiquidCrystal.h"

// =======================================================
//                     PIN ASSIGNMENTS
// =======================================================

// DC motors (L293D, 1-direction)
const int motorL = 6;   // Left shuffle roller
const int motorR = 7;   // Right shuffle roller
const int motorD = 8;   // Dealing roller
const int motorS = 9;   // Shooting roller

// Stepper 28BYJ-48 + ULN2003
#define IN1 10
#define IN2 11
#define IN3 12
#define IN4 13

// Buttons (original wiring maintained)
const int butPlayUp   = 2;
const int butPlayDown = 3;
const int butCardUp   = 4;
const int butCardDown = 5;
const int butStart    = A2;

// Photoresistors
const int phoRight = A0;
const int phoLeft  = A1;

// Random seed
const int randPin = A3;

// =======================================================
//                     CONSTANTS
// =======================================================

const unsigned long cardDelay = 300;
const int sensorThreshold = 500;

// Original values
int numPlayers = 4;
int numCards   = 52;

// LCD (16-pin parallel)
Adafruit_LiquidCrystal lcd(0);

// =======================================================
//                  STEPPER (AccelStepper)
// =======================================================

// Best torque mode, correct sequence
AccelStepper stepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

const float stepsPerRev = 2048.0;
const float stepsPerDegree = stepsPerRev / 360.0;

float currentAngle = 0.0;
int stepperSpeed = 1000;   // Default speed (you asked for a “speed constant variable”)

// =======================================================
//                     HELPER FUNCTIONS
// =======================================================

bool leftHasCards()  { return analogRead(phoLeft)  < sensorThreshold; }
bool rightHasCards() { return analogRead(phoRight) < sensorThreshold; }

void runMotor(int pin) {
  digitalWrite(pin, HIGH);
  delay(cardDelay);
  digitalWrite(pin, LOW);
}

// ===== ANIMATIONS =====
void lcdShufflingAnimation() {
  lcd.clear();
  lcd.print("Shuffling...");
  Serial.println("Shuffling animation");

  for (int i = 0; i < 3; i++) {
    lcd.setCursor(0, 1);
    lcd.print(".   ");
    delay(250);
    lcd.setCursor(0, 1);
    lcd.print("..  ");
    delay(250);
    lcd.setCursor(0, 1);
    lcd.print("...");
    delay(250);
  }
}

void lcdDealingAnimation() {
  lcd.clear();
  lcd.print("Dealing cards");
  Serial.println("Dealing animation");

  for (int i = 0; i < 5; i++) {
    lcd.setCursor(0, 1);
    lcd.print(">");
    delay(150);
    lcd.setCursor(0, 1);
    lcd.print(">>");
    delay(150);
    lcd.setCursor(0, 1);
    lcd.print(">>>");
    delay(150);
    lcd.setCursor(0, 1);
    lcd.print("    ");
    delay(150);
  }
}

void updateLCD() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Players: "); lcd.print(numPlayers);
  lcd.setCursor(0,1); lcd.print("Cards:   "); lcd.print(numCards);
}

// =======================================================
//                  STEPPER CONTROL
// =======================================================

void setSpeed(int spd) {
  stepperSpeed = constrain(spd, 200, 1800);
  stepper.setMaxSpeed(stepperSpeed);
}

void rotateDegrees(float degrees) {
  float diff = degrees;

  // Select shortest path
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  long steps = diff * stepsPerDegree;
  stepper.move(steps);
  currentAngle += diff;

  // Wrap
  if (currentAngle >= 360) currentAngle -= 360;
  if (currentAngle < 0)    currentAngle += 360;
}

// =======================================================
//                  SERIAL TEST MODE (Manual Control)
// =======================================================

void runTestMode() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("TEST MODE");
  lcd.setCursor(0,1);
  lcd.print("Serial Control");

  Serial.println("\n===== TEST MODE =====");
  Serial.println("Motors:");
  Serial.println("  L1/L0  = Left Motor ON/OFF");
  Serial.println("  R1/R0  = Right Motor ON/OFF");
  Serial.println("  D1/D0  = Dealing Motor ON/OFF");
  Serial.println("  S1/S0  = Shooter Motor ON/OFF");
  Serial.println("Stepper:");
  Serial.println("  CW      = rotate +20 deg");
  Serial.println("  CCW     = rotate -20 deg");
  Serial.println("  SPD###  = set speed");
  Serial.println("Sensors:");
  Serial.println("  READ    = read photoresistors");
  Serial.println("=======================");

  while (true) {

    stepper.run();

    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();

      // ----------- Motor Commands -----------
      if (cmd == "L1") { digitalWrite(motorL, HIGH); Serial.println("Motor L ON"); }
      if (cmd == "L0") { digitalWrite(motorL, LOW);  Serial.println("Motor L OFF"); }

      if (cmd == "R1") { digitalWrite(motorR, HIGH); Serial.println("Motor R ON"); }
      if (cmd == "R0") { digitalWrite(motorR, LOW);  Serial.println("Motor R OFF"); }

      if (cmd == "D1") { digitalWrite(motorD, HIGH); Serial.println("Motor D ON"); }
      if (cmd == "D0") { digitalWrite(motorD, LOW);  Serial.println("Motor D OFF"); }

      if (cmd == "S1") { digitalWrite(motorS, HIGH); Serial.println("Motor S ON"); }
      if (cmd == "S0") { digitalWrite(motorS, LOW);  Serial.println("Motor S OFF"); }

      // ----------- Stepper Commands -----------
      if (cmd == "CW") {
        Serial.println("Stepper: +90 deg");
        rotateDegrees(90);
      }
      if (cmd == "CCW") {
        Serial.println("Stepper: -20 deg");
        rotateDegrees(-90);
      }

      if (cmd.startsWith("SPD")) {
        int sp = cmd.substring(3).toInt();
        setSpeed(sp);
        Serial.print("Stepper speed set to: ");
        Serial.println(stepperSpeed);
      }

      // ----------- Sensor Readings -----------
      if (cmd == "READ") {
        Serial.print("Left  = "); Serial.println(analogRead(phoLeft));
        Serial.print("Right = "); Serial.println(analogRead(phoRight));
      }
    }
  }
}

// =======================================================
//                        SETUP
// =======================================================

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(randPin));

  pinMode(motorL, OUTPUT);
  pinMode(motorR, OUTPUT);
  pinMode(motorD, OUTPUT);
  pinMode(motorS, OUTPUT);

  pinMode(butPlayUp,   INPUT_PULLUP);
  pinMode(butPlayDown, INPUT_PULLUP);
  pinMode(butCardUp,   INPUT_PULLUP);
  pinMode(butCardDown, INPUT_PULLUP);
  pinMode(butStart,    INPUT_PULLUP);

  lcd.begin(16,2);

  // Stepper initial speed
  stepper.setMaxSpeed(stepperSpeed);
  stepper.setAcceleration(800);

  updateLCD();
}

// =======================================================
//                        LOOP
// =======================================================

void loop() {

  // ------------------- TEST MODE VIA SERIAL -------------------
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "TEST") {
      runTestMode();
    }
  }

  // ------------------- PLAYER BUTTONS -------------------
  if (!digitalRead(butPlayUp)) {
    numPlayers++;
    updateLCD();
    delay(200);
  }

  if (!digitalRead(butPlayDown)) {
    if (numPlayers > 1) numPlayers--;
    updateLCD();
    delay(200);
  }

  // ------------------- CARD BUTTONS -------------------
  if (!digitalRead(butCardUp)) {
    numCards++;
    updateLCD();
    delay(200);
  }

  if (!digitalRead(butCardDown)) {
    if (numCards > 1) numCards--;
    updateLCD();
    delay(200);
  }

  // ------------------- START BUTTON -------------------
  if (!digitalRead(butStart)) {

    // LCD animation before shuffling
    lcdShufflingAnimation();

    bool left  = leftHasCards();
    bool right = rightHasCards();

    // SHUFFLING
    while (left || right) {
      if (left && right) {
        int r = random(0,2);
        if (r == 0) runMotor(motorL);
        else        runMotor(motorR);
      }
      else if (left)  runMotor(motorL);
      else if (right) runMotor(motorR);

      delay(55);

      left  = leftHasCards();
      right = rightHasCards();
    }

    // DEALING
    lcdDealingAnimation();

    float angle = 360.0 / numPlayers;

    digitalWrite(motorS, HIGH);

    int remaining = numCards;
    while (remaining > 0) {

      runMotor(motorD);   // Deal card
      rotateDegrees(angle);

      remaining--;
    }

    digitalWrite(motorS, LOW);

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("  Finished!");
    delay(1500);

    updateLCD();
  }

  stepper.run();
}
    