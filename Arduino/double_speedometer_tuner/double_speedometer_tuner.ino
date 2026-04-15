//run this code see how far the vehicle goes and how much it reports use this to calculate the effective wheel size
volatile long turns = 0; // total turns difference
volatile long dist = 0;  // total pulses
volatile unsigned long lastPulseLeft = 0;
volatile unsigned long lastPulseRight = 0;
volatile float speedLeft = 0;
volatile float speedRight = 0;
long timer=0;
int throttle_speed = 110;   //90 is stopped higher numbers faster. currently set very slow for testing.
#include <Servo.h>// This Library includes the control functions to interface with RC car Hobby components
//meters per speedometer pulse at 2 pulses this is π*r where r is the radius of the wheel
#define distancePerPulse (PI * 0.03)
Servo steering;  // create servo object to control steering. this is the front servo
Servo throttle;  //create another servo object to control throttle this actually controls the esc

void setup() {
    steering.attach(D4);  // attaches the servo on pin 4 to the steering object
  throttle.attach(D8); //attaches the ESC on pin 8 to the throttle object
  throttle.write(90);
  Serial.begin(9600);

  pinMode(D2, INPUT); // left wheel
  pinMode(D3, INPUT); // right wheel
  //placing the left wheel on 2 and the right one on 3 prevents these wires from crossing
  attachInterrupt(digitalPinToInterrupt(D2), leftWheelISR, RISING);
  attachInterrupt(digitalPinToInterrupt(D3), rightWheelISR, RISING);
}
//ISRs for speedometer are separate in order to save time on execution
// ISR for left wheel
void leftWheelISR() {
  unsigned long now = micros();
  if (lastPulseLeft != 0) {
    unsigned long dt = now - lastPulseLeft; // microseconds between pulses
    speedLeft = distancePerPulse / (dt / 1e6); // m/s
  }
  lastPulseLeft = now;
  
  dist++;
  turns++;
}

// ISR for right wheel
void rightWheelISR() {
  unsigned long now = micros();
  if (lastPulseRight != 0) {
    unsigned long dt = now - lastPulseRight;
    speedRight = distancePerPulse / (dt / 1e6); // m/s
  }
  lastPulseRight = now;

  dist++;
  turns--;
}
void drive(){//this function controls the main driving opperations which must be done repeatedly.
 throttle.write(throttle_speed);
 steering.write(90); 
}
void loop() {
  // Copy volatile variables safely
  noInterrupts();
  float leftSpeed = speedLeft;
  float rightSpeed = speedRight;
  long t = turns;
  long d = dist;
  interrupts();
  if(digitalRead(D5)){
    if(timer=0){
      timer=millis()
    }if(timer-millis()>10*1000){
    throttle.write(90)
    Serial1.print("Dist: ");
    Serial1.print(d);
    Serial1.print(", Turns: ");
    Serial1.println(t);
    return;
    }
    drive();
  } else{
  Serial1.print("ready");
  }
  delay(100);
}