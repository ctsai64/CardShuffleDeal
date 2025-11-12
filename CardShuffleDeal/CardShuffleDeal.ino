#include <Stepper.h>
#include "Adafruit_LiquidCrystal.h"

// ===== PIN ASSIGNMENTS =====
const int motorL = 6;
const int motorR = 7;
const int motorD = 8;   
const int motorS = 9;

const int stp1 = 10, stp2 = 11, stp3 = 12, stp4 = 13;
Stepper stepperMotor(200, stp1, stp2, stp3, stp4);

const int butPlayUp    = 2;
const int butPlayDown  = 3;
const int butCardsUp   = 4;
const int butCardsDown = 5;
const int butStart     = A2;

const int phoLeft  = A0;
const int phoRight = A1;

Adafruit_LiquidCrystal lcd(0); // clk A5, dat A4

// ===== GAME VARIABLES =====
int numPlayers = 4;
int numCards = 52;

const int cardDelay = 200;
const int sensorThreshold = 300;

float currentAngle = 0; // stepper tracking

bool stopped = false;  // NEW — track stop state

// ===== SETUP =====
void setup() {
  Serial.begin(9600);

  pinMode(motorL, OUTPUT);
  pinMode(motorR, OUTPUT);
  pinMode(motorD, OUTPUT);
  pinMode(motorS, OUTPUT);

  pinMode(butPlayUp, INPUT_PULLUP);
  pinMode(butPlayDown, INPUT_PULLUP);
  pinMode(butCardsUp, INPUT_PULLUP);
  pinMode(butCardsDown, INPUT_PULLUP);
  pinMode(butStart, INPUT_PULLUP);

  stepperMotor.setSpeed(30);

  lcd.begin(16, 2);
  lcd.clear();
  updateLCD();
  stopAllMotors();
  Serial.println("=== System Ready ===");
}

// ===== UTILITY =====
void stopAllMotors() {
  digitalWrite(motorL, LOW);
  digitalWrite(motorR, LOW);
  digitalWrite(motorD, LOW);
  digitalWrite(motorS, LOW);
}

// ===== LCD UPDATE =====
void updateLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Players: ");
  lcd.print(numPlayers);
  lcd.setCursor(0, 1);
  lcd.print("Cards:   ");
  lcd.print(numCards);
}

// ===== STOP HANDLER =====
void handleStop() {
  stopAllMotors();
  stopped = true;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("=== STOPPED ===");
  Serial.println("Processes stopped!");
  delay(1000);
  stopped = false;
  updateLCD();
}

// ===== DEBUGGING =====
void checkDebugCommand() {
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') continue;

    if (cmd == 'D' || cmd == 'd') {
      Serial.println("=== DEBUG INFO ===");
      Serial.print("Players: "); Serial.println(numPlayers);
      Serial.print("Cards: "); Serial.println(numCards);
      Serial.print("Button UP: "); Serial.println(!digitalRead(butPlayUp));
      Serial.print("Button DOWN: "); Serial.println(!digitalRead(butPlayDown));
      Serial.print("Button Cards UP: "); Serial.println(!digitalRead(butCardsUp));
      Serial.print("Button Cards DOWN: "); Serial.println(!digitalRead(butCardsDown));
      Serial.print("Button START: "); Serial.println(!digitalRead(butStart));
      Serial.print("Left Card Sensor: "); Serial.println(analogRead(phoLeft));
      Serial.print("Motor L: "); Serial.println(digitalRead(motorL));
      Serial.print("Motor R: "); Serial.println(digitalRead(motorR));
      Serial.print("Motor D/S: "); Serial.println(digitalRead(motorD));
      Serial.print("Stepper angle: "); Serial.println(currentAngle);
      Serial.println("==================");
    }
  }
}

// ===== SHUFFLE =====
bool shuffleStage() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Shuffling...");
  Serial.println("Starting shuffle...");

  char spinner[] = {'|', '/', '-', '\\'};
  int spinIndex = 0;

  bool leftCards = analogRead(phoLeft) < sensorThreshold;

  while (leftCards) {
    if (!digitalRead(butStart)) { // emergency stop
      Serial.println("Shuffle stopped by button!");
      handleStop();
      return false;
    }

    digitalWrite(motorL, HIGH);
    digitalWrite(motorR, HIGH);
    delay(cardDelay);
    digitalWrite(motorL, LOW);
    digitalWrite(motorR, LOW);

    // spinner animation
    lcd.setCursor(14, 0);
    lcd.print(spinner[spinIndex]);
    spinIndex = (spinIndex + 1) % 4;

    leftCards = analogRead(phoLeft) < sensorThreshold;
  }

  stopAllMotors();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Shuffle Done!");
  Serial.println("Shuffle complete.");
  delay(500);
  return true;
}

// ===== DEAL =====
bool dealStage() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dealing...");
  Serial.println("Starting deal...");

  float anglePerPlayer = 360.0 / numPlayers;
  int barLength = 16;
  int filled = 0;

  for (int i = 0; i < numCards; i++) {
    if (!digitalRead(butStart)) { // emergency stop
      Serial.println("Deal stopped by button!");
      handleStop();
      return false;
    }

    digitalWrite(motorD, HIGH);
    digitalWrite(motorS, HIGH);
    delay(cardDelay);
    digitalWrite(motorD, LOW);
    digitalWrite(motorS, LOW);

    stepperMotor.step(round(200 * (anglePerPlayer / 360.0)));
    currentAngle += anglePerPlayer;
    if (currentAngle >= 360) currentAngle -= 360;

    // progress bar animation
    int newFill = map(i + 1, 0, numCards, 0, barLength);
    if (newFill > filled) {
      lcd.setCursor(0, 1);
      for (int j = 0; j < newFill; j++) lcd.print((char)255);
      for (int j = newFill; j < barLength; j++) lcd.print(' ');
      filled = newFill;
    }
  }

  stopAllMotors();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Deal Done!");
  Serial.println("Deal complete.");
  delay(800);
  updateLCD();
  return true;
}

// ===== MAIN LOOP =====
void loop() {
  checkDebugCommand();

  // Menu buttons
  if (!digitalRead(butPlayUp)) { 
    Serial.println("Button: PlayUp pressed");
    numPlayers++; updateLCD(); delay(200); 
  }
  if (!digitalRead(butPlayDown) && numPlayers > 1) { 
    Serial.println("Button: PlayDown pressed");
    numPlayers--; updateLCD(); delay(200); 
  }
  if (!digitalRead(butCardsUp)) { 
    Serial.println("Button: CardsUp pressed");
    numCards++; updateLCD(); delay(200); 
  }
  if (!digitalRead(butCardsDown) && numCards > 1) { 
    Serial.println("Button: CardsDown pressed");
    numCards--; updateLCD(); delay(200); 
  }

  // Start button
  if (!digitalRead(butStart)) {
    Serial.println("Button: Start pressed");
    delay(50); // debounce
    if (!digitalRead(butStart)) {
      if (!shuffleStage()) return;  // stop if aborted
      if (!dealStage()) return;     // stop if aborted
    }
  }
}
