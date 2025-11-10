#include <AccelStepper.h>

// ULN2003 + 28BYJ-48 wiring
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9

// Strongest torque stepping mode
AccelStepper stepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

// 28BYJ-48 true gear ratio → accurate steps
const float accurateStepsPerRev = 2048.0;
const float stepsPerDegree = accurateStepsPerRev / 360.0;

float currentAngle = 0.0;
int currentSpeed = 800; // default speed

void setup() {
  Serial.begin(9600);
  Serial.println("Stepper Ready!");
  Serial.println("Commands: A### for angle 0–360, S### for speed");
  stepper.setMaxSpeed(currentSpeed);
  stepper.setAcceleration(800);
}

void loop() {
  stepper.run();

  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'A' || cmd == 'a') {
      float angle = Serial.parseFloat();
      setAngle(angle);
    }
    else if (cmd == 'S' || cmd == 's') {
      setSpeed(Serial.parseInt());
    }
  }
}

void setAngle(float target) {
  // Wrap into 0–360
  target = fmod(target, 360.0);
  if (target < 0) target += 360.0;

  // Find shortest rotation direction
  float diff = target - currentAngle;
  if (diff > 180) diff -= 360;
  if (diff < -180) diff += 360;

  long steps = diff * stepsPerDegree;
  stepper.move(steps);

  currentAngle = target;

  Serial.print("→ Target: ");
  Serial.print(target, 1);
  Serial.println("°");
}

void setSpeed(int spd) {
  currentSpeed = constrain(spd, 100, 2000);
  stepper.setMaxSpeed(currentSpeed);

  Serial.print("✔ Speed: ");
  Serial.print(currentSpeed);
  Serial.println(" steps/sec");
}
