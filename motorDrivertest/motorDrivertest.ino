/*
  DRV8833_Motor_Control.ino (Simple On/Off)

  Controls two DC motors with a DRV8833 motor driver
  using only two digital pins for simple ON/OFF control.

  - This code provides ON/OFF control only (no reverse).
  - Speed is at 100%.

  Serial Commands:
  --------------------------
  'a' = Motor A: ON
  's' = Motor A: OFF

  'j' = Motor B: ON
  'k' = Motor B: OFF
  --------------------------

  *** DRV8833 WIRING (On/Off) ***
  - DRV8833 VCC     -> Arduino 5V
  - DRV8833 VM      -> 6-9V Motor Power Supply (+)
  - DRV8833 GND     -> Common GND (Arduino GND + Motor Supply GND)
  - DRV8833 nSLEEP  -> Arduino 5V (to enable the chip)
  
  - DRV8833 AIN1    -> Arduino Pin 6 (Motor A ON/OFF)
  - DRV8833 AIN2    -> **GND** (Tie to Ground)
  - DRV8833 AOUT1/2 -> Motor A

  - DRV8833 BIN1    -> Arduino Pin 7 (Motor B ON/OFF)
  - DRV8833 BIN2    -> **GND** (Tie to Ground)
  - DRV8833 BOUT1/2 -> Motor B
*/

// Motor A Control Pin (as requested)
const int motorA_pin = 6;

// Motor B Control Pin (as requested)
const int motorB_pin = 7;


void setup() {
  // Set all motor pins as OUTPUTs
  pinMode(motorA_pin, OUTPUT);
  pinMode(motorB_pin, OUTPUT);

  // Start Serial communication at 9600 baud
  Serial.begin(9600);

  // Print instructions to the Serial Monitor
  Serial.println("--- DRV8833 Simple On/Off Control ---");
  Serial.println("Motor A: 'a'(ON), 's'(OFF)");
  Serial.println("Motor B: 'j'(ON), 'k'(OFF)");
  Serial.println("-----------------------------------");
  
  // Start with both motors stopped
  stopAllMotors();
}

void loop() {
  // Check if there is any data from the Serial Monitor
  if (Serial.available() > 0) {
    // Read the incoming command
    char command = Serial.read();

    // Remove any other stray characters from the buffer
    while(Serial.available() > 0) { Serial.read(); }

    // --- Process the command ---
    switch (command) {
      // --- Motor A Controls ---
      case 'a':
        motorA_on();
        Serial.println("Motor A: ON");
        break;
      case 's':
        motorA_off();
        Serial.println("Motor A: OFF");
        break;

      // --- Motor B Controls ---
      case 'j':
        motorB_on();
        Serial.println("Motor B: ON");
        break;
      case 'k':
        motorB_off();
        Serial.println("Motor B: OFF");
        break;

      // --- Default Case ---
      default:
        Serial.println("Unknown command. Use: a,s,j,k");
        break;
    }
  }
}

// --- Helper Functions for Motor Control ---

// --- Motor A ---
void motorA_on() {
  digitalWrite(motorA_pin, HIGH);
}

void motorA_off() {
  digitalWrite(motorA_pin, LOW);
}

// --- Motor B ---
void motorB_on() {
  digitalWrite(motorB_pin, HIGH);
}

void motorB_off() {
  digitalWrite(motorB_pin, LOW);
}

// --- Utility ---
void stopAllMotors() {
  motorA_off();
  motorB_off();
}