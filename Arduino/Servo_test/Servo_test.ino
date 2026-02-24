#define min_angle 87 //minimum angle in degrees
#define max_angle 100 //maximum angle in degrees
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 90;    // variable to store the servo position

void setup() {
  myservo.attach(D4);  // attaches the servo on pin 4 to the servo object
}

void loop() {
  for (; pos <= max_angle; pos += 1) { // goes from min degrees to max degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(60);                       // waits 15ms for the servo to reach the position
  }
  for (; pos >= min_angle; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(30);                       // waits 15ms for the servo to reach the position
  }
}