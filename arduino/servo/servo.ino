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

void setup() {
    // We need to attach the servo to the used pin number
    Yaw.attach(yawPin);
    Pitch.attach(pitchPin);
    Trigger.attach(triggerPin);

    Yaw.write(90);
    Pitch.write(90);                  // init pitch to default position
    Trigger.writeMicroseconds(2500);  // init trigger to default position
    
    Serial.begin(250000);
    Serial.setTimeout(1);
}

void fire() {
    // firing servo has 0-270 degree swing corresponding to 500 micro-sec - 2500 micro-sec PWM
    Trigger.writeMicroseconds(2500);
    delay(350);
    Trigger.writeMicroseconds(2000);
    delay(350);
    Trigger.writeMicroseconds(2500);
}

void movePitch(int angle) {
    if (angle > 125) angle = 125;
    if (angle < 60) angle = 55;
    Pitch.write(angle);
}

void moveYaw(int angle) {
    // 0 degrees points to right
    // 180 degrees points to left
    Yaw.write(angle);
}

void loop() {
  while (!Serial.available());
  int serial_in = Serial.readString().toInt();

  if (serial_in) {
    byte yaw = serial_in >> 0 & 0b11111111;
    byte pitch = serial_in >> 8 & 0b11111111;
  }
}
