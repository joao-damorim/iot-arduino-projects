#include <Stepper.h>

#define motorPin1  5      // IN1 on the ULN2003 driver
#define motorPin2  0      // IN2 on the ULN2003 driver
#define motorPin3  14     // IN3 on the ULN2003 driver
#define motorPin4  13     // IN4 on the ULN2003 driver

const int stepsPerRevolution = 2050;

Stepper stepper1(stepsPerRevolution, motorPin1, motorPin2, motorPin3, motorPin4);

void setup() {
  Serial.begin(9600);
  // Set the maximum steps per second:
  stepper1.setSpeed(10);
}

void loop() {
  stepper1.step(1);
}
