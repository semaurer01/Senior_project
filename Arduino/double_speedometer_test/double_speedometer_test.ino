volatile long turns = 0; // total turns difference
volatile long dist = 0;  // total pulses
volatile unsigned long lastPulseLeft = 0;
volatile unsigned long lastPulseRight = 0;
volatile float speedLeft = 0;
volatile float speedRight = 0;
//meters per speedometer pulse at 2 pulses per revolution this is π*r where r is the radius of the wheel
float distancePerPulse = PI*.07;

void setup() {
  Serial.begin(9600);

  pinMode(2, INPUT); // left wheel
  pinMode(3, INPUT); // right wheel

  attachInterrupt(digitalPinToInterrupt(2), leftWheelISR, RISING);
  attachInterrupt(digitalPinToInterrupt(3), rightWheelISR, RISING);
}

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

void loop() {
  // Copy volatile variables safely
  noInterrupts();
  float leftSpeed = speedLeft;
  float rightSpeed = speedRight;
  long t = turns;
  long d = dist;
  interrupts();

  Serial.print("Turns: "); Serial.print(t);
  Serial.print(", Dist: "); Serial.print(d);
  Serial.print(", Left Speed: "); Serial.print(leftSpeed);
  Serial.print(" m/s, Right Speed: "); Serial.println(rightSpeed);

  delay(100); // update display
}