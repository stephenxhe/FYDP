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

    Yaw.write(0);
    Pitch.write(90);                  // init pitch to default position
    Trigger.writeMicroseconds(2500);  // init trigger to default position
    Serial.begin(9600);
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
    moveYaw(60);
    delay(500);
    fire();
    delay(2000);
    moveYaw(120);
    delay(500);
    fire();
    delay(2000);
    moveYaw(90);
    delay(500);
    fire();
    delay(2000);

    movePitch(100);
    delay(500);
    fire();
    delay(2000);
    movePitch(80);
    delay(500);
    fire();
    delay(2000);
    movePitch(90);
    delay(500);
    fire();
    delay(2000);
}
