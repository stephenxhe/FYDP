//#include <AFMotor.h>
#include <Servo.h>
//const int stepsPerRevolution = 200;
//const float degreeToStep = 1.8;

AF_Stepper pitchMotor(stepsPerRevolution, 1);
AF_Stepper yawMotor(stepsPerRevolution, 2);

void setup() {
  Serial.begin(250000);
  Serial.setTimeout(1);
  yawMotor.setSpeed(5);  // 10 rpm
  pitchMotor.setSpeed(5);
}

void loop() {
//  Serial.readStringUntil('\n');
  while (!Serial.available());
  
  int serial_in = Serial.readString().toInt();

  if (serial_in) {
    byte yaw = serial_in >> 0 & 0b11111111;
    byte pitch = serial_in >> 8 & 0b11111111;
    if (pitch<50 && pitch>0){
      if (pitch > 25) {
        pitchMotor.step((pitch-25)/degreeToStep, BACKWARD, INTERLEAVE); 
      } else if (pitch < 25) {
        pitchMotor.step((25-pitch)/degreeToStep, FORWARD, INTERLEAVE); 
      }
    }
    
    if (yaw<90 && yaw>0){
      if (yaw > 45) {\
        yawMotor.step((yaw-45)/degreeToStep, BACKWARD, INTERLEAVE); 
      } else if (yaw < 45) {
        yawMotor.step((45-yaw)/degreeToStep, FORWARD, INTERLEAVE); 
      }
    }
  }
}
