#include <AccelStepper.h>

#define pinEnable 8
#define dirPin 13
#define stepPin 12

// Define the AccelStepper interface type:

#define motorInterfaceType2 1

AccelStepper stepper2 = AccelStepper(motorInterfaceType2, stepPin, dirPin);

void setup() {
  Serial.begin(9600);
  //digitalWrite(pinEnable, HIGH);
  // Set the maximum steps per second:
  stepper2.setMaxSpeed(1000);
  // Set the speed of the motor in steps per second:
  stepper2.setSpeed(34);
}

void loop() {
  // Step the motor with constant speed as set by setSpeed():
  stepper2.runSpeed();
}
