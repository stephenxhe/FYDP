#include <AFMotor.h>
const int stepsPerRevolution = 200;
const float degreeToStep = 1.8;
AF_Stepper yawMotor(stepsPerRevolution, 1);

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  yawMotor.setSpeed(5);  // 10 rpm
  delay(2000);
}

int yaw;
int cur_x;
int cur_y;
int global_x = 90;
int global_y = 0;// set cur position to global center
bool running = true;

void loop() {
  while (!Serial.available());

  if (running == true) {
    yaw = (Serial.readString().toInt());
  
    if (yaw<90 && yaw>0){
      if (yaw > 45) {
//        cur_x = global_x + (yaw-45);
        if (cur_x < 180 || true){
          global_x = cur_x;
          yawMotor.step((yaw-45)/degreeToStep, BACKWARD, INTERLEAVE); 
        }     
      } else if (yaw < 45) {
//        cur_x = global_x - (45-yaw);
        if (cur_x > 0 || true){
          global_x = cur_x;
          yawMotor.step((45-yaw)/degreeToStep, FORWARD, INTERLEAVE); 
        }
      }
    } 
  }
}
