#include <AFMotor.h>
#include <AFMotor.h>
const int stepsPerRevolution = 200;
//each step is 1.8 degrees 
//for steps divide angle by steps/degree
const float degreeToStep = 1.8;
AF_Stepper motor(stepsPerRevolution, 2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(1);

  motor.setSpeed(25);  // 10 rpm
  delay(2000);
}

int x;
int position = 45; // track degrees away from center [0, 90]

void loop() {
  while (!Serial.available());
  x = (Serial.readString().toInt());
  
  if (x<90 && x>0){
    if (x > 45) {
      position += x-45;
      motor.step((x-45)/degreeToStep, BACKWARD, SINGLE);
    } else if (x < 45) {
      position -= 45-x;
      motor.step((45-x)/degreeToStep, FORWARD, SINGLE); 
    }
  }
}
