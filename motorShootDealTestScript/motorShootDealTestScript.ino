// --- DRV8833 Dual Motor Driver Controller for Arduino Mega 2560 ---
// Control Motors via Serial Input: 
//   - CS<speed>: Change Shooting Speed (0-255)
//   - CD<speed>: Change Dealing Speed (0-255)
//   - S: Start Shooting Roller (Continuous)
//   - X: Stop Shooting Roller
//   - d<a>s<b>b<c>s<d>: Dealing Pulse with forward/backward time & speed

// --- Pin Definitions ---
const int motorD1 = 8;  
const int motorD2 = 9;  
const int motorS1 = 6;  
const int motorS2 = 7;  

int shootingSpeed = 255; 
int dealingSpeed = 255;

bool isShootingRollerOn = false; 

void setMotorSpeed(int pinIN1, int pinIN2, int speed, int direction) {
  speed = constrain(speed, 0, 255);

  if (speed == 0) {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
  } 
  else if (direction > 0) {
    analogWrite(pinIN1, speed);
    digitalWrite(pinIN2, LOW);
  } 
  else if (direction < 0) {
    digitalWrite(pinIN1, LOW);
    analogWrite(pinIN2, speed);
  }
}

void runDealingRoller(int speed, int direction) {
  Serial.print("Dealing Roller Speed: ");
  Serial.println(speed);
  setMotorSpeed(motorD1, motorD2, speed, direction);
}

void runShootingRoller(int speed, int direction) {
  Serial.print("Shooting Roller Speed: ");
  Serial.println(speed);
  setMotorSpeed(motorS1, motorS2, speed, direction);
}

void stopDealingRoller() {
  Serial.println("Stopping Dealing Roller.");
  setMotorSpeed(motorD1, motorD2, 0, 0);
}

void stopShootingRoller() {
  Serial.println("Stopping Shooting Roller.");
  setMotorSpeed(motorS1, motorS2, 0, 0);
}

void setup() {
  Serial.begin(9600);
  Serial.println("DRV8833 Motor Controller Ready.");

  pinMode(motorD1, OUTPUT);
  pinMode(motorD2, OUTPUT);
  pinMode(motorS1, OUTPUT);
  pinMode(motorS2, OUTPUT);

  stopDealingRoller();
  stopShootingRoller();
}

void loop() {
  if (Serial.available() > 0) {

    char command = toupper(Serial.read());

    // -------- CS / CD SPEED COMMANDS --------
    if (command == 'C') {
      if (Serial.available()) {
        char id = toupper(Serial.read());
        long v = Serial.parseInt();
        if (v < 0 || v > 255) {
          Serial.println("Speed must be 0–255");
          while (Serial.available()) Serial.read();
          return;
        }
        if (id == 'S') {
          shootingSpeed = v;
          if (isShootingRollerOn) runShootingRoller(shootingSpeed, 1);
          Serial.print("Shooting speed set: ");
          Serial.println(shootingSpeed);
        }
        else if (id == 'D') {
          dealingSpeed = v;
          Serial.print("Dealing speed set: ");
          Serial.println(dealingSpeed);
        }
        else Serial.println("Unknown ID. Use CS or CD.");
      }
      while (Serial.available()) Serial.read();
      return;
    }

    // -------- CONTINUOUS SHOOTING --------
    if (command == 'S') {
      runShootingRoller(shootingSpeed, 1);
      isShootingRollerOn = true;
      Serial.println("Shooting Roller Started.");
      while (Serial.available()) Serial.read();
      return;
    }

    if (command == 'X') {
      stopShootingRoller();
      isShootingRollerOn = false;
      Serial.println("Shooting Roller Stopped.");
      while (Serial.available()) Serial.read();
      return;
    }

    // ==========================================================
    //            NEW DEALING COMMAND FORMAT
    //               d<a>s<b>b<c>s<d>
    // ==========================================================
    if (command == 'D') {

      long forwardTime = Serial.parseInt();  

      if (Serial.read() != 's') { Serial.println("ERR: Expected s"); return; }

      long forwardSpeed = Serial.parseInt();

      if (Serial.read() != 'b') { Serial.println("ERR: Expected b"); return; }

      long backwardTime = Serial.parseInt();

      if (Serial.read() != 's') { Serial.println("ERR: Expected s"); return; }

      long backwardSpeed = Serial.parseInt();

      // --- VALIDATE ---
      if (forwardTime < 0 || backwardTime < 0) {
        Serial.println("ERR: times must be positive");
        return;
      }
      forwardSpeed = constrain(forwardSpeed, 0, 255);
      backwardSpeed = constrain(backwardSpeed, 0, 255);

      Serial.println("Parsed command:");
      Serial.print("Forward Time: "); Serial.println(forwardTime);
      Serial.print("Forward Speed: "); Serial.println(forwardSpeed);
      Serial.print("Backward Time: "); Serial.println(backwardTime);
      Serial.print("Backward Speed: "); Serial.println(backwardSpeed);

      // ---------- RUN FORWARD ----------
      Serial.println("Running Forward...");
      runDealingRoller(forwardSpeed, 1);
      delay(forwardTime);
      stopDealingRoller();
      delay(25);

      // ---------- RUN BACKWARD ----------
      Serial.println("Running Backward...");
      runDealingRoller(backwardSpeed, -1);
      delay(backwardTime);
      stopDealingRoller();

      Serial.println("Done.");
      while (Serial.available()) Serial.read();
      return;
    }

    // ----- UNKNOWN COMMAND -----
    Serial.print("Unknown command: ");
    Serial.println(command);
    while (Serial.available()) Serial.read();
  }

  delay(10);
}
