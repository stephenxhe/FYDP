#include <AFMotor.h>
const int stepsPerRevolution = 200;
const float degreeToStep = 1.8;
AF_Stepper yawMotor(stepsPerRevolution, 1);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
 
  yawMotor.setSpeed(10);  // 10 rpm
  delay(2000);
}

int yaw;
int position = 45; // track degrees away from center [0, 90]

void loop() {
  while (!Serial.available());
  yaw = (Serial.readString().toInt());
  
  if (yaw<90 && yaw>0){
    if (yaw > 45) {
      position += yaw-45;
      yawMotor.step((yaw-45)/degreeToStep, BACKWARD, SINGLE);
    } else if (yaw < 45) {
      position -= 45-yaw;
      yawMotor.step((45-yaw)/degreeToStep, FORWARD, SINGLE); 
    }
  }
}
