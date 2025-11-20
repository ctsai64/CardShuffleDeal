#include <AccelStepper.h>
#include "Adafruit_LiquidCrystal.h"

// =======================================================
//                   PIN ASSIGNMENTS
// =======================================================

// DC motors (L293D, 1-direction)
const int motorL1 = 9;  // Left shuffle roller control pin 1 (Your list: 9 - motorL1)
const int motorL2 = 8;  // Left shuffle roller control pin 2 (Your list: 8 - motorL2)
const int motorR1 = 7;  // Right shuffle roller control pin 1 (Your list: 7 - motorR1)
const int motorR2 = 6;  // Right shuffle roller control pin 2 (Your list: 6 - motorR2)
const int motorD1 = 5;  // Dealing roller control pin 1 (Your list: 5 - motorD1)
const int motorD2 = 4;  // Dealing roller control pin 2 (Your list: 4 - motorD2)
const int motorS1 = 3;  // Shooting roller control pin 1 (Your list: 3 - motorS1)
const int motorS2 = 2;  // Shooting roller control pin 2 (Your list: 2 - motorS2)

// Stepper 28BYJ-48 + ULN2003
#define IN1 10
#define IN2 11
#define IN3 12
#define IN4 13

// Buttons (Digital and Analog)
const int butPlayUp   = A2; 
const int butPlayDown = A3; 
const int butStart    = A4; 
const int butCardUp   = A5;  
const int butCardDown = A6;  

// Photoresistors
const int phoLeft  = A1; // A1 - pho2 -> renamed to phoLeft for clarity
const int phoRight = A0; // A0 - pho1 -> renamed to phoRight for clarity

// Random seed
const int randPin = A7;
// =======================================================
//                     CONSTANTS
// =======================================================

const unsigned long cardDelay = 300;
const int sensorThreshold = 1;

// DC Motor Speed Constants (0-255)
const int motorSpeedL = 255; // Left motor speed
const int motorSpeedR = 255; // Right motor speed
const int motorSpeedD = 255; // Dealing motor speed
const int motorSpeedS = 255; // Shooting motor speed

// Original values
int numPlayers = 4;
int numCards   = 52;

// LCD (16-pin parallel)
Adafruit_LiquidCrystal lcd(0); //Data 20, Clock 21

// =======================================================
//                  STEPPER (AccelStepper)
// =======================================================

// Best torque mode, correct sequence (Based on your pins: 10, 12, 11, 13)
AccelStepper stepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

const float stepsPerRev = 2048.0;
const float stepsPerDegree = stepsPerRev / 360.0;

float currentAngle = 0.0;
int stepperSpeed = 1000;  // Default speed (you asked for a “speed constant variable”)

// =======================================================
//                    HELPER FUNCTIONS
// =======================================================

bool leftHasCards()  { return analogRead(phoLeft)  < sensorThreshold; }
bool rightHasCards() { return analogRead(phoRight) < sensorThreshold; }

// Updated to use the new motor pin constants (assuming motorL1/motorL2 etc. are paired)
// Note: This function now takes two pins for the motor pair (motor control and direction/brake,
// but the original code only ran one pin HIGH. I'm keeping the original logic but using the D1/R1/L1/S1 pins.)
void runMotor(int pin) {
  digitalWrite(pin, HIGH);
  delay(cardDelay);
  digitalWrite(pin, LOW);
}

// ===== ANIMATIONS =====
void lcdShufflingAnimation() {
  lcd.clear();
  lcd.print("Shuffling...");

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
//                    STEPPER CONTROL
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
//             SERIAL TEST MODE (Manual Control)
// =======================================================

void runTestMode() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("TEST MODE");
  lcd.setCursor(0,1);
  lcd.print("Serial Control");

  Serial.println("\n===== TEST MODE =====");
  Serial.println("Motors (Control pin 1):");
  Serial.println("  L1/L0  = Left Motor ON/OFF");
  Serial.println("  R1/R0  = Right Motor ON/OFF");
  Serial.println("  D1/D0  = Dealing Motor ON/OFF");
  Serial.println("  S1/S0  = Shooter Motor ON/OFF");
  Serial.println("Stepper:");
  Serial.println("  CW      = rotate +90 deg");
  Serial.println("  CCW     = rotate -90 deg");
  Serial.println("  SPD###  = set speed");
  Serial.println("Sensors:");
  Serial.println("  READ    = read photoresistors");
  Serial.println("=======================");

  while (true) {

    stepper.run();

    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();

      // ----------- Motor Commands (using motor *1 pins) -----------
      // NOTE: For L293D use, you may need to set motor *2 pin LOW
      if (cmd == "L1") { digitalWrite(motorL1, HIGH); Serial.println("Motor L ON"); }
      if (cmd == "L0") { digitalWrite(motorL1, LOW);  Serial.println("Motor L OFF"); }

      if (cmd == "R1") { digitalWrite(motorR1, HIGH); Serial.println("Motor R ON"); }
      if (cmd == "R0") { digitalWrite(motorR1, LOW);  Serial.println("Motor R OFF"); }

      if (cmd == "D1") { digitalWrite(motorD1, HIGH); Serial.println("Motor D ON"); }
      if (cmd == "D0") { digitalWrite(motorD1, LOW);  Serial.println("Motor D OFF"); }

      if (cmd == "S1") { digitalWrite(motorS1, HIGH); Serial.println("Motor S ON"); }
      if (cmd == "S0") { digitalWrite(motorS1, LOW);  Serial.println("Motor S OFF"); }

      // ----------- Stepper Commands -----------
      if (cmd == "CW") {
        Serial.println("Stepper: +90 deg");
        rotateDegrees(90);
      }
      if (cmd == "CCW") {
        Serial.println("Stepper: -90 deg");
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

  // Motor pins are now paired
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(motorD1, OUTPUT);
  pinMode(motorD2, OUTPUT);
  pinMode(motorS1, OUTPUT);
  pinMode(motorS2, OUTPUT);

  // Set default direction for DC motors (pin *2 LOW for forward direction, assuming L293D/H-bridge)
  digitalWrite(motorL2, LOW);
  digitalWrite(motorR2, LOW);
  digitalWrite(motorD2, LOW);
  digitalWrite(motorS2, LOW);
  
  // Note: If you want to use the motor speed constants, you'll need to use 
  // analogWrite() instead of digitalWrite(pin, HIGH) and use PWM-capable pins (3, 5, 6, 9, 10, 11).
  
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
//                         LOOP
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
    // Limits
    if (numPlayers < 8) numPlayers++;
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
    // Limits
    if (numCards < 100) numCards++;
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

    // SHUFFLING (Using *1 pins, assuming *2 pins are LOW for forward)
    while (left || right) {
      Serial.println("Have cards, shuffling");
      if (left && right) {
        int r = random(0,2);
        if (r == 0) runMotor(motorL1); // Use motorL1 pin
        else        runMotor(motorR1); // Use motorR1 pin
      }
      else if (left)  runMotor(motorL1);
      else if (right) runMotor(motorR1);

      delay(55);

      left  = leftHasCards();
      right = rightHasCards();
    }

    // DEALING
    lcdDealingAnimation();

    float angle = 360.0 / numPlayers;

    digitalWrite(motorS1, HIGH); // Shooter motor ON

    int remaining = numCards;
    while (remaining > 0) {
      Serial.println("Dealing");
      runMotor(motorD1); // This runs the dealing motor and includes a delay(cardDelay)

      rotateDegrees(angle);
      
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }

      remaining--;
    }

    digitalWrite(motorS1, LOW); // Shooter motor OFF

    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("   Finished!");
    delay(1500);

    updateLCD();
  }

  stepper.run();
}
