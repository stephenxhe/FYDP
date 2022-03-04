// Include the Servo library
#include <Servo.h>

// Declare the Servo pins
int yawPin = 9;
int pitchPin = 7;
int triggerPin = 6;

// Create servo objects
Servo Yaw;
Servo Pitch;
Servo Trigger;

// create variables to store the servo positions
int currentYaw;
int currentPitch;

void setup() {
    // We need to attach the servo to the used pin number
    Yaw.attach(yawPin);
    Pitch.attach(pitchPin);
//    Trigger.attach(triggerPin);

    currentYaw = 90;        // init yaw to default position
    currentPitch = 90;      // init pitch to default position
//    Trigger.writeMicroseconds(2500);  // init trigger to default position

    Yaw.write(currentYaw);
    Pitch.write(100);                  
    
    Serial.begin(250000);
    Serial.setTimeout(1);
}

//void fire() {
//    // firing servo has 0-270 degree swing corresponding to 500 micro-sec - 2500 micro-sec PWM
//    Trigger.writeMicroseconds(2500);
//    delay(350);
//    Trigger.writeMicroseconds(2000);
//    delay(350);
//    Trigger.writeMicroseconds(2500);
//}

void movePitch(int angle, int motorSpeed) {
  // limit pitch fov     
  if (angle > 125) angle = 125;
  if (angle < 55) angle = 55;

  if (currentPitch < angle) {
    for (int i = currentPitch; i < angle + 5; i++) {
      Pitch.write(i);
      delay(motorSpeed);
    }
  } else if (currentPitch > angle) {
    for (int i = currentPitch; i > angle - 5; i--) {
      Pitch.write(i);
      delay(motorSpeed);
    }
  }

  currentPitch = Pitch.read();
}

void moveYaw(int angle, int motorSpeed) {
  // 0 degrees points to right
  // 180 degrees points to left

  // limit yaw fov
  if (angle > 135) angle = 135;
  if (angle < 45) angle = 45;
  
  if (currentYaw < angle) {
    for (int i = currentYaw; i < angle - 0; i++) {
      Yaw.write(i);
      delay(motorSpeed);
    }
  } else if (currentYaw > angle) {
    for (int i = currentYaw; i > angle + 0; i--) {
      Yaw.write(i);
      delay(motorSpeed);
    }
  }
  currentYaw = Yaw.read();
}

void loop() {
  while (!Serial.available());
  
  int serialIn = Serial.readString().toInt();
  int yawIn = serialIn >> 0 & 0b11111111;
  int pitchIn = serialIn >> 8 & 0b11111111;

  if (yawIn<90 && yawIn>0){
    int newYaw = currentYaw + 45 - yawIn;
//    if (abs(newYaw - currentYaw) > 5) {
    moveYaw(newYaw, 10); 
//    } else {
//      moveYaw(currentYaw, 10);
//    }
  }

//    if (pitchIn<50 && pitchIn>0){
//      int newPitch = currentPitch + 25 - pitchIn;
//      movePitch(newPitch, 10);
//    }
  Serial.println(pitchIn); 
}