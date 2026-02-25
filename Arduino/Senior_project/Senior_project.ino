#define min_steering_angle 87 //minimum angle in degrees
#define max_steering_angle 100 //maximum angle in degrees
#include <Servo.h>

Servo steering;  // create servo object to control steering. this is the front servo
Servo throttle;  //create another servo object to control throttle this actually controls the esc
// twelve servo objects can be created on most boards

int steering_angle = 90;    // variable to store the servo position
int throttle_speed = 87;  //90 is stopped lower numbers faster. currently set very slow for testing

void setup() {
  steering.attach(D4);  // attaches the servo on pin 4 to the steering object
  throttle.attach(D8); //attaches the ESC on pin 8 to the throttle object
}

void drive(){//this function controls the main driving opperations which must be done repeatedly.
  steering.write(steering_angle);//set the steering angle
  Throttle.write(throttle_speed);//set the speed
}

void loop() {
  // put your main code here, to run repeatedly:
  drive();
}
