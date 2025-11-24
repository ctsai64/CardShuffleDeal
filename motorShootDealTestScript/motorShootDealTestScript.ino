// --- DRV8833 Dual Motor Driver Controller for Arduino Mega 2560 ---
// Control Motors via Serial Input: 
//   - CS<speed>: Change Shooting Speed (0-255)
//   - CD<speed>: Change Dealing Speed (0-255)
//   - S: Start Shooting Roller (Continuous)
//   - X: Stop Shooting Roller
//   - D<A>B<B>: Dealing Roller Pulse (A=forward ms, B=backward ms). Ex: D1500B500

// --- Pin Definitions ---
const int motorD1 = 5;  // Dealing Roller IN1 (A-IN1)
const int motorD2 = 4;  // Dealing Roller IN2 (A-IN2)

const int motorS1 = 3;  // Shooting Roller IN1 (B-IN1)
const int motorS2 = 2;  // Shooting Roller IN2 (B-IN2)

// --- Global Speed Variables (Adjustable via Serial) ---
int shootingSpeed = 255; 
int dealingSpeed = 200;

// --- Global State Variables ---
// Tracks the current state of the Shooting Roller for the 'S' and 'X' commands.
bool isShootingRollerOn = false; 


/**
 * @brief Sets the speed and direction of a single DC motor connected to the DRV8833.
 * @param pinIN1 The Arduino pin connected to the motor driver's IN1.
 * @param pinIN2 The Arduino pin connected to the motor driver's IN2.
 * @param speed The PWM speed value (0 = stop, 255 = full speed).
 * @param direction 1 for forward (IN1 PWM, IN2 LOW), -1 for reverse (IN2 PWM, IN1 LOW).
 */
void setMotorSpeed(int pinIN1, int pinIN2, int speed, int direction) {
  // Ensure speed is within the valid range
  speed = constrain(speed, 0, 255);

  if (speed == 0) {
    // If speed is 0, brake/stop the motor (Coast/Slow stop is when both are LOW)
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
  } else if (direction > 0) {
    // Forward direction
    analogWrite(pinIN1, speed);
    digitalWrite(pinIN2, LOW);
  } else if (direction < 0) {
    // Reverse direction
    digitalWrite(pinIN1, LOW);
    analogWrite(pinIN2, speed);
  }
}

// --- Motor Control Helper Functions ---

void runDealingRoller(int speed, int direction) {
  Serial.print("Running Dealing Roller at speed: ");
  Serial.println(speed);
  setMotorSpeed(motorD1, motorD2, speed, direction);
}

void runShootingRoller(int speed, int direction) {
  Serial.print("Running Shooting Roller at speed: ");
  Serial.println(speed);
  setMotorSpeed(motorS1, motorS2, speed, direction);
}

void stopDealingRoller() {
  Serial.println("Stopping Dealing Roller.");
  setMotorSpeed(motorD1, motorD2, 0, 0); // Speed 0 stops the motor
}

void stopShootingRoller() {
  Serial.println("Stopping Shooting Roller.");
  setMotorSpeed(motorS1, motorS2, 0, 0); // Speed 0 stops the motor
}


void setup() {
  // Initialize Serial communication for debugging
  Serial.begin(9600);
  Serial.println("DRV8833 Motor Controller Ready.");
  Serial.print("Current Speeds: Shooting=");
  Serial.print(shootingSpeed);
  Serial.print(", Dealing=");
  Serial.println(dealingSpeed);
  Serial.println("Commands:");
  Serial.println("  CS<speed> -> Change Shooting Speed (0-255). Ex: CS180");
  Serial.println("  CD<speed> -> Change Dealing Speed (0-255). Ex: CD100");
  Serial.println("  S         -> Start Shooting Roller (Continuous)");
  Serial.println("  X         -> Stop Shooting Roller");
  Serial.println("  D<A>B<B>  -> Dealing Pulse: A ms Forward, B ms Backward. Ex: D1500B500");
  
  // Set all motor control pins as outputs
  pinMode(motorD1, OUTPUT);
  pinMode(motorD2, OUTPUT);
  pinMode(motorS1, OUTPUT);
  pinMode(motorS2, OUTPUT);

  // Ensure motors are stopped initially
  stopDealingRoller();
  stopShootingRoller();
}

void loop() {
  // Check if a command is available in the serial buffer
  if (Serial.available() > 0) {
    // Read the incoming command character (e.g., 'C', 'S', 'X', or 'D')
    char command = Serial.read();
    command = toupper(command); 

    // --- Speed Change Commands (CS and CD) ---
    if (command == 'C') {
        if (Serial.available() > 0) {
            char motorID = toupper(Serial.read()); // Read S or D
            long newSpeed = Serial.parseInt(); // Read the number
            
            // Check if the number is in the valid PWM range
            if (newSpeed >= 0 && newSpeed <= 255) {
                if (motorID == 'S') {
                    shootingSpeed = (int)newSpeed;
                    Serial.print("Shooting speed set to: ");
                    Serial.println(shootingSpeed);
                    // If the shooting motor is currently running, update its speed instantly
                    if (isShootingRollerOn) {
                        runShootingRoller(shootingSpeed, 1);
                    }
                } else if (motorID == 'D') {
                    dealingSpeed = (int)newSpeed;
                    Serial.print("Dealing speed set to: ");
                    Serial.println(dealingSpeed);
                } else {
                    Serial.println("Error: Unknown motor ID for speed change. Use CS<speed> or CD<speed>.");
                }
            } else {
                Serial.println("Error: Speed must be between 0 and 255.");
            }
            // Clear remaining buffer after speed command
            while (Serial.available()) { Serial.read(); }
            return;
        }
    }

    // --- Continuous Commands (S and X) ---
    if (command == 'S') {
      runShootingRoller(shootingSpeed, 1); // Uses current shootingSpeed
      isShootingRollerOn = true;
      Serial.println("Shooting Roller started continuously.");
      // Clear buffer for simple commands
      while (Serial.available()) { Serial.read(); }
      return; 
    }
    
    if (command == 'X') {
      stopShootingRoller();
      isShootingRollerOn = false;
      Serial.println("Shooting Roller stopped.");
      // Clear buffer for simple commands
      while (Serial.available()) { Serial.read(); }
      return;
    }

    // --- Timed Pulse Command (D<A>B<B>) ---
    if (command == 'D') {
      long durationForward = Serial.parseInt(); // Reads duration 'A'
      
      // Look for the 'B' character immediately after the first number
      char separator = 0; 
      if (Serial.peek() == 'B' || Serial.peek() == 'b') {
          separator = Serial.read(); // Consume the 'B'
      }
      
      long durationBackward = Serial.parseInt(); // Reads duration 'B'

      if (durationForward > 0 && durationBackward >= 0 && toupper(separator) == 'B') {
        Serial.print("Command received: D");
        Serial.print(durationForward);
        Serial.print("B");
        Serial.print(durationBackward);
        Serial.println(" ms pulse sequence.");

        // --- Forward Pulse (A) ---
        Serial.print("Forward pulse (A) for "); Serial.print(durationForward); Serial.println(" ms.");
        runDealingRoller(dealingSpeed, 1); // Direction 1: Forward
        delay(durationForward); 
        stopDealingRoller();

        // --- Backward Pulse (B) ---
        if (durationBackward > 0) {
            // Short delay to avoid issues when rapidly reversing current on the motor driver
            delay(25); 
            Serial.print("Backward pulse (B) for "); Serial.print(durationBackward); Serial.println(" ms.");
            runDealingRoller(dealingSpeed, -1); // Direction -1: Reverse
            delay(durationBackward); 
            stopDealingRoller();
        }

        Serial.println("Dealing Roller sequence complete.");

      } else {
        Serial.println("Error: Command format must be D<forward_ms>B<backward_ms>. E.g., D1500B500.");
      }
      
      // Clear any remaining serial data
      while (Serial.available()) {
        Serial.read();
      }
      
    } else {
      // Handle unrecognized command
      Serial.print("Unrecognized command: ");
      Serial.println(command);
      // Clear all serial data if an unrecognized command is found
      while (Serial.available()) {
        Serial.read();
      }
    }
  }

  // A small delay for general loop stability
  delay(10);
}