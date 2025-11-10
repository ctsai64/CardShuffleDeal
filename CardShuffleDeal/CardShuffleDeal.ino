#include <Stepper.h>
#include <LiquidCrystal.h>

// ===== PIN ASSIGNMENTS =====
const int motorL = 2;
const int motorR = 3;
const int motorD = 0;   
const int motorS = 0;

const int stp1 = 6, stp2 = 7, stp3 = 8, stp4 = 9;
Stepper stepperMotor(200, stp1, stp2, stp3, stp4);

const int butPlayUp    = A2;
const int butPlayDown  = A3;
const int butCardsUp   = A4;
const int butCardsDown = A5;
const int butStart     = A1;

const int phoLeft  = A0;
const int phoRight = 0;  // pin conflict, ignored

LiquidCrystal lcd(4, 5, 10, 11, 12, 13);

// ===== GAME VARIABLES =====
int numPlayers = 4;
int numCards = 52;

const int cardDelay = 200;
const int sensorThreshold = 300;

float currentAngle = 0; // stepper tracking

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
  lcd.begin(16,2);
  updateLCD();
  stopAllMotors();
}

// ===== UTILITY =====
void stopAllMotors() {
  digitalWrite(motorL, LOW);
  digitalWrite(motorR, LOW);
  digitalWrite(motorD, LOW);
  digitalWrite(motorS, LOW);
}

void updateLCD() {
  lcd.clear();
  lcd.print("Players: "); lcd.print(numPlayers);
  lcd.setCursor(0,1);
  lcd.print("Cards: "); lcd.print(numCards);
}

// ===== DEBUGGING (one-shot) =====
void checkDebugCommand() {
  while(Serial.available() > 0) {
    char cmd = Serial.read();
    if(cmd == '\n' || cmd == '\r') continue;

    if(cmd == 'D' || cmd == 'd') {
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
void shuffleStage() {
  lcd.clear(); lcd.print("Shuffling...");
  bool leftCards = analogRead(phoLeft) < sensorThreshold;

  while(leftCards) {
    if(!digitalRead(butStart)) { // emergency stop
      stopAllMotors();
      updateLCD();
      return;
    }

    digitalWrite(motorL,HIGH);
    digitalWrite(motorR,HIGH);
    delay(cardDelay);
    digitalWrite(motorL,LOW);
    digitalWrite(motorR,LOW);

    leftCards = analogRead(phoLeft) < sensorThreshold;
  }

  stopAllMotors();
  lcd.clear(); lcd.print("Shuffle Done!");
  delay(300);
}

// ===== DEAL =====
void dealStage() {
  lcd.clear(); lcd.print("Dealing...");
  float anglePerPlayer = 360.0 / numPlayers;

  for(int i=0;i<numCards;i++){
    if(!digitalRead(butStart)) { // emergency stop
      stopAllMotors();
      updateLCD();
      return;
    }

    digitalWrite(motorD,HIGH);
    digitalWrite(motorS,HIGH);
    delay(cardDelay);
    digitalWrite(motorD,LOW);
    digitalWrite(motorS,LOW);

    stepperMotor.step(round(200 * (anglePerPlayer / 360.0)));
    currentAngle += anglePerPlayer;
    if(currentAngle >= 360) currentAngle -= 360;

    lcd.setCursor(0,1);
    lcd.print("Card -> Player "); lcd.print((i % numPlayers)+1);
  }

  stopAllMotors();
  lcd.clear(); lcd.print("Deal Done!");
  delay(300);
  updateLCD();
}

// ===== MAIN LOOP =====
void loop() {
  checkDebugCommand();

  // Menu buttons
  if(!digitalRead(butPlayUp)){ numPlayers++; updateLCD(); delay(200);}
  if(!digitalRead(butPlayDown) && numPlayers>1){ numPlayers--; updateLCD(); delay(200);}
  if(!digitalRead(butCardsUp)){ numCards++; updateLCD(); delay(200);}
  if(!digitalRead(butCardsDown) && numCards>1){ numCards--; updateLCD(); delay(200);}

  // Start button
  if(!digitalRead(butStart)) {
    delay(50); // debounce
    if(!digitalRead(butStart)) {
      shuffleStage();
      dealStage();
    }
  }
}
