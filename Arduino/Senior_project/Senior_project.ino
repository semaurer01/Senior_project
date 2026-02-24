#define min_steering_angle 87 //minimum angle in degrees
#define max_steering_angle 100 //maximum angle in degrees
#include <Servo.h>

Servo steering;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int steering_angle = 90;    // variable to store the servo position

void setup() {
  steering.attach(D4);  // attaches the servo on pin 4 to the servo object
}

void drive(){//this function controls the main driving opperations which must be done repeatedly.
  steering.write(steering_angle);
}

void loop() {
  // put your main code here, to run repeatedly:
  
}
