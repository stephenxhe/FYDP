#include <AFMotor.h>
#include <AFMotor.h>
const int stepsPerRevolution = 200;
//each step is 1.8 degrees 
//for steps divide angle by steps/degree
const float degreeToStep = 1.8;
AF_Stepper motor2(stepsPerRevolution, 1);
AF_Stepper motor1(stepsPerRevolution, 2);

const float fov_vertical = 51/2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(1);
 
  motor1.setSpeed(25);
//  motor2.setSpeed(25);  // 10 rpm
  delay(2000);
}

int x;
int position = 51/2; // track degrees away from center [0, 90]

void loop() {
  while (!Serial.available());
  x = (Serial.readString().toInt());
  
  if (x<fov_vertical*2 && x>0){
    if (x > fov_vertical) {
      position += x-fov_vertical;
      motor1.step((x-fov_vertical)/degreeToStep, FORWARD, DOUBLE);
    } else if (x < fov_vertical) {
      position -= fov_vertical-x;
      motor1.step((fov_vertical-x)/degreeToStep, BACKWARD, DOUBLE); 
    }
  }
}
