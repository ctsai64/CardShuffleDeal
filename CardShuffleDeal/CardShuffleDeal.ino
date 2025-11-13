#include <Stepper.h>
#include "Adafruit_LiquidCrystal.h"

// ===== PIN ASSIGNMENTS =====
// DRV8833 outputs for stepper motor
const int stp1 = 10;
const int stp2 = 11;
const int stp3 = 12;
const int stp4 = 13;

Stepper stepperMotor(200, stp1, stp2, stp3, stp4);

// ===== BUTTONS =====
const int butPlayUp    = 2;
const int butPlayDown  = 3;
const int butCardsUp   = 4;
const int butCardsDown = 5;
const int butStart     = A2;

// ===== LCD =====
Adafruit_LiquidCrystal lcd(0); // uses A4/A5 for I2C or shift register

// ===== VARIABLES =====
int numPlayers = 4;
int numCards = 52;
bool running = false;
bool stopped = false;

void setup() {
  Serial.begin(9600);

  // Pins
  pinMode(butPlayUp, INPUT_PULLUP);
  pinMode(butPlayDown, INPUT_PULLUP);
  pinMode(butCardsUp, INPUT_PULLUP);
  pinMode(butCardsDown, INPUT_PULLUP);
  pinMode(butStart, INPUT_PULLUP);

  // Stepper
  stepperMotor.setSpeed(40);

  // LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Players: ");
  lcd.print(numPlayers);
  lcd.setCursor(0, 1);
  lcd.print("Cards: ");
  lcd.print(numCards);
}

void loop() {
  // ===== BUTTON DEBUG =====
  if (digitalRead(butStart) == LOW) {
    delay(150); // debounce
    running = !running;
    if (running) {
      Serial.println("== Starting shuffle/deal ==");
      shuffleAnimation();
      dealAnimation();
      running = false;
    } else {
      Serial.println("== Stopped ==");
      stopped = true;
      displayStopped();
      delay(2000);
      stopped = false;
      updateLCD();
    }
  }
}

// ===== LCD =====
void updateLCD() {
  lcd.clear();
  lcd.print("Players: ");
  lcd.print(numPlayers);
  lcd.setCursor(0, 1);
  lcd.print("Cards: ");
  lcd.print(numCards);
}

void displayStopped() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("== STOPPED ==");
  Serial.println("Display: STOPPED");
}

// ===== ANIMATIONS =====
void shuffleAnimation() {
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

    // small motor wiggle to simulate shuffle
    stepperMotor.step(50);
    stepperMotor.step(-50);
  }
}

void dealAnimation() {
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

    stepperMotor.step(100);  // simulate dealing motion
  }
}
