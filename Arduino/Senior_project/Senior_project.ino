//steering constants
#define min_steering_angle 60 //minimum angle in degrees
#define max_steering_angle 110 //maximum angle in degrees
#define steering_center 80 //the angle offset that is strait.
#define max_squared_distance_for_waypoint (1.2*1.2) //meters from target before target switches
#define throttle_max 125
#define throttle_stable 112
bool throttle_started = 0;
//speedometer constants

//meters per speedometer pulse at 2 pulses this is π*r where r is the radius of the wheel
#define distancePerPulse (PI * 0.03)
#define speedometer_confidence 0.1//how much to change speed estimate based on speedometer values each iteration of the predict phase
//GPS
#define start_location_confidence 0.99 //how sure am I that this is the current location (changes regularly)
float start_latitude   = 40.604637;//estimate of initial location
float start_longitude = -83.124848;//estimate of initial location
#define earth_radius_meters 6361632 
#define degree_to_meter (earth_radius_meters*PI/180)
float cosine_latitude = cos(radians(start_latitude));//doesn't need to change dynamically because machine will always be near start.
// MPU6050 constants
#define MPU6050_ADDR 0x69  // MPU6050 I2C address
#define PWR_MGMT_1   0x6B  // Power management register
#define MPU6050_BEGINING 0x3B  // Accelerometer data registers
#define ACCEL_CONFIG 0x1C  // Accelerometer configuration register
#define GYRO_CONFIG 0x1B   // Gyroscope configuration register
#define ACCEL_CONFIG_VALUE 0  // sets limits to 2g
#define GYRO_CONFIG_VALUE 0  // sets limits to 250°/s
#define GYRO_UNITS_TO_RADIANS (PI/(180.0f*131.0f))
#define ACCEL_UNITS_TO_MS2 (9.81 / 16384.0)

//Kalman Constants
#define minimum_speed_for_kalman_update 2 // m/s if the vehicle is not moving the gps data will be inaccurate
#define max_speed_resonable 30
#define kalman_states 5 //number of variables tracked
#define kalman_measurements 4 //number of data used in the kalman update function
//libraries
#include <Servo.h>// This Library includes the control functions to interface with RC car Hobby components
#include <Wire.h> // this library connects the IMU and GPS via IIC
#include "SparkFun_I2C_GPS_Arduino_Library.h" // https://github.com/sparkfun/SparkFun_I2C_GPS_Arduino_Library
#include "TinyGPSPlus.h"

//set the course data with an array of waypoints in meters {latitudinal difference,} longitudinal difference 
//generated with the waypoint_generator.js
/*original float waypoints[][2]={
  {0,0},
	{-6.096,11.07283464566929},
	{-7.239000000000001,-8.202099737532809},
	{-6.3627,-122.21128608923884},
	{-1.6383,-84.48162729658792},
	{-0.11430000000000001,-101.29593175853017},
	{-6.5532,-180.85629921259843},
	{-7.124700000000001,-203.001968503937},
	{6.8199000000000005,-209.97375328083987},
	{6.1341,-97.19488188976378},
	{7.124700000000001,9.02230971128609},
	{0,0},
};*/
//adapted because orientation was wrong manually calculated values
float waypoints[][2]={
  {0,0},
	{-2,-7},
	{0,-9},
	{10,-7},
	{11.5,-6.5},
	{10.2,-3.5},
	{7,-2.5},
  {10,2},
  {20.2,-9.5},
  {23,-8},
  {20.2,-3},
  {20.2,3},
  {23,8},
  {20.2,9.5},
  {15,-7},
  {5,-7},
  {0,-9},
  {-1,-6},
  {0,0},
  {0,5}
};
//navigation variables 
volatile unsigned long lastPulseLeft = 0;
volatile unsigned long lastPulseRight = 0;
volatile float speedLeft = 0;
volatile float speedRight = 0;
int seeking=0; // the waypoint that the machine is tyring to reach
Servo steering;  // create servo object to control steering. this is the front servo
Servo throttle;  //create another servo object to control throttle this actually controls the esc
// twelve servo objects can be created on most boards
I2CGPS myI2CGPS; // I2C object for communication
TinyGPSPlus gps;//TinyGps object for interpretation
//control variables
int steering_angle = 90;    // variable to store the servo position
int throttle_speed = 90;   //90 is stopped higher numbers faster. currently set very slow for testing.
uint8_t mode=0;// the status of the machine currently only records whether the machine should be driving
//data variables 
int16_t mpu_data [7];//stores raw data from the mpu

float gps_data [4];//gps latitude longitude speed direction

// ===== KALMAN FILTER STATES=====
float x[kalman_states][1]={
  {0},//heading
  {0},//gyro_bias
  {0},//speed
  {0},//longitude
  {0} //latitude
};

// Covariance matrix (3x3) these values are currently made up and have no basis in experamentation
float P[kalman_states][kalman_states] = {
  {1, 0, 0, 0, 0},
  {0, 1, 0, 0, 0},
  {0, 0, 5, 0, 0},
  {0 ,0, 0, 2, 0},
  {0, 0, 0, 0, 2}
};

// Process noise
static float Q[]={
    0.05,    // gyro noise
    0.003,   // gyro bias drift
    0.003,     // speed process noise
    0.01, //longtitudinal uncertainty
    0.01 //latitudinal uncertainty
};
// Measurement noise
float R_gps_heading = 1.0;  // GPS heading noise
float R_gps_speed   = 0.15;  // GPS speed noise
float R_gps_x       =0.1; //gps latitude noise
float R_gps_y       =0.1; //gps longitude noise

unsigned long lastTime = 0; //timing the difference between gps updates

void setup() {
  //steering and drive
  steering.attach(D4);  // attaches the servo on pin 4 to the steering object
  throttle.attach(D8); //attaches the ESC on pin 8 to the throttle object
  throttle.write(90);
  int throttle_start=millis();
  //Bluetooth/debugging
  Serial1.begin(9600);//connect to the bluetooth module over UART

  //sensors
  //MPU
  Wire.begin();
  Wire.setClock(100000);// 100kHz
  // Wake up the MPU6050 as it starts in sleep mode
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);  // Set the PWR_MGMT_1 register to 0 to wake up the sensor
  int error=Wire.endTransmission(false);
  if(error){// if the mpu is invalid stop and report it.
    bluetooth(2,error);
    while(1);
  }
  // Configure the accelerometer and gyroscope
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_CONFIG);          // Select the accelerometer configuration register
  Wire.write(ACCEL_CONFIG_VALUE);    // Set the accelerometer to 2g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_CONFIG);           // Select the gyroscope configuration register
  Wire.write(GYRO_CONFIG_VALUE);     // Set the gyroscope to 250°/s
  Wire.endTransmission(true);
  //GPS
  while (!myI2CGPS.begin(Wire1)) {//until connected to gps report error 2 (no connection)
    bluetooth(3,-1);
    delay(500);
  }
  Wire1.setClock(400000); // 400 kHz
    // Build the PMTK packet for 1 Hz (1000 ms)
  String config = myI2CGPS.createMTKpacket(220, ",1000"); 
  // Send it
  myI2CGPS.sendMTKpacket(config);

  //input settings
  pinMode(D5,INPUT);//switch 1
  
  pinMode(D2, INPUT); // left wheel
  pinMode(D3, INPUT); // right wheel
  //placing the left wheel on 2 and the right one on 3 prevents these wires from crossing
  attachInterrupt(digitalPinToInterrupt(D2), leftWheelISR, RISING);
  attachInterrupt(digitalPinToInterrupt(D3), rightWheelISR, RISING);
  //last thing before loop start timer
  while(millis()-throttle_start<1000);
  lastTime =micros();
}
void bluetooth(int status,int error){ //debugging data this will do nothing during competition 
  switch(status){
    case 0:// for when nothing is happening probably an error state
        Serial1.print("idling in state ");
        Serial1.println(error);
        Serial1.print(start_latitude,6);
        Serial1.print(", ");
        Serial1.println(start_longitude,6);
        return;
    case 1://debug MPU
        Serial1.println("mpu update:");
        for(int i=0; i<7;i++){
          Serial1.print(mpu_data[i]);
          Serial1.print(", ");
        }
        Serial1.println();
        return;
    case 2://when there is something wrong with the mpu
      Serial1.print("mpu error: ");
      Serial1.println(error);
      return;
    case 3://GPS error
      Serial1.print("gps error: ");
      Serial1.println(error);
      return;
    case 4://GPS update
      Serial1.println("gps update:"); 
      Serial1.print(gps.satellites.value());
      Serial1.print(", ");
      for(int i=0; i<3;i++){
          Serial1.print(gps_data[i]);
          Serial1.print(", ");
        }
        Serial.println();
    return;
    case 5://Kalman output
      Serial1.println("kalman Update");
      Serial1.print(gps.satellites.value());
      Serial1.print(", ");
      Serial1.print(x[3][0]);
      Serial1.print(", ");
      Serial1.print(x[4][0]);
      Serial1.print(", ");
      Serial1.print(x[2][0]);
      Serial1.print(", ");
      Serial1.println(x[0][0]);
      bluetooth(4,0);
    return;
    case 6://kalman error checking
      for(int i=0;i<kalman_states;i++)//valid covariance
        for(int j=0;j<kalman_states;j++)
          if(isnan(P[i][j])){
            Serial1.print("Invalid covariance at:P(");
            Serial1.print(i);
            Serial1.print(",");
            Serial1.println(j);
          }
      for(int i=0;i<kalman_states;i++){//positive diagonals
        if(P[i][i]<0){
          Serial1.print("Negative Covariance:");
          Serial1.println(i);
          return;
          }
      }
      if(abs(x[0][0]-gps_data[3])>30){//error to much
        Serial1.print("heading estimate off: ");
        Serial1.println(x[0][0]-gps_data[3]);
        return;
      }
      
    return;
    case 7:
    Serial1.print("math error");
    Serial1.println(error);
  }//end switch

  Serial1.println("invalid bluetooth");
}

void drive(){//this function controls the main driving opperations which must be done repeatedly.
  float dy = waypoints[seeking][0] - x[4][0]; // latitude
  float dx = waypoints[seeking][1] - x[3][0]; // longitude
  while((dy*dy+dx*dx)<max_squared_distance_for_waypoint){
    seeking++;
    dy = waypoints[seeking][0] - x[4][0];
    dx = waypoints[seeking][1] - x[3][0];
  }
  float head=atan2(dx,dy);
  float error = head - x[0][0];

  // wrap error
  if(error > PI) error -= 2*PI;
  if(error < -PI) error += 2*PI;

  steering_angle = (steering_center + degrees(error))*.1+steering_angle*.9;

  steering_angle = constrain(steering_angle,
                           min_steering_angle,
                           max_steering_angle);
  steering.write(steering_angle);//set the steering angle
  if(digitalRead(D5)){
    throttle_speed=throttle_max;
    throttle.write(throttle_speed);//set the speed
    
    if(throttle_speed>=throttle_max){
      throttle_started=1;
    }
    if(throttle_speed<throttle_max&& !throttle_started ){
      throttle_speed++;
    }else if( throttle_speed>throttle_stable){
      throttle_speed--;
    }
  }else{
    throttle_started=0;
    throttle_speed=90;
    throttle.write(90);
  }
}

void mpu(){
  Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_BEGINING);  // Start reading from the first accelerometer register
    int error=Wire.endTransmission(true);
    if(error){
      bluetooth(2,error);//if the error is not zero complain
      return;
    }
    int received=Wire.requestFrom(MPU6050_ADDR, 14);  // Request 14 bytes (6 for accelerometer, 6 for gyroscope, 2 temparature currently ignored)
    if(received<14){
      bluetooth(2,received+100);//costom error for no characters recieved
      return;
    };
  for(int i=0;i<7;i++){//loop through the data 
    mpu_data[i]=((Wire.read() << 8) | Wire.read());
  }
}
bool gps_update(){
    // Read all available bytes from GPS
    int bytesAvailable = myI2CGPS.available();

  for (int i = 0; i < bytesAvailable; i++) {
    char c = myI2CGPS.read();
    gps.encode(c);
  }
  //if there is no data return
  if(!gps.location.isUpdated() &&
     !gps.speed.isUpdated() &&
     !gps.course.isUpdated()) return false;
  //if data is invalid do not use
  if(gps.location.isValid()){
    gps_data[0]=(gps.location.lat()-start_latitude)*degree_to_meter;
    gps_data[1]=(gps.location.lng()-start_longitude)*degree_to_meter*cosine_latitude;  
  }
  if(gps.speed.isValid()) gps_data[2]=gps.speed.mps();
  if(gps.course.isValid())  gps_data[3]=radians(gps.course.deg());
  //update data 
  //return valid if there is new data
  return true;
}

//ISRs for speedometer are separate in order to save time on execution{
  // ISR for left wheel
  void leftWheelISR() {
    unsigned long now = micros();
    if (lastPulseLeft != 0) {
      unsigned long dt = now - lastPulseLeft; // microseconds between pulses
      speedLeft = distancePerPulse / (dt / 1e6); // m/s
    }
    lastPulseLeft = now;
  }

  // ISR for right wheel
  void rightWheelISR() {
    unsigned long now = micros();
    if (lastPulseRight != 0) {
      unsigned long dt = now - lastPulseRight;
      speedRight = distancePerPulse / (dt / 1e6); // m/s
    }
    lastPulseRight = now;
  }
//}

bool mat_inverse(float A[][kalman_measurements], float A_inv[][kalman_measurements], int m)
{
    // Create augmented matrix [A | I]
    float aug[kalman_measurements][2*kalman_measurements];

    for(int i=0;i<m;i++){
        for(int j=0;j<m;j++)
            aug[i][j] = A[i][j];

        for(int j=0;j<m;j++)
            aug[i][j+m] = (i==j) ? 1.0f : 0.0f;
    }

    // Gauss-Jordan elimination
    for(int i=0;i<m;i++)
    {
        float pivot = aug[i][i];

        if (fabs(pivot) < 1e-6){
            bluetooth(7,0);
            return false; // singular matrix
            
        }
        // Normalize pivot row
        for(int j=0;j<2*m;j++)
            aug[i][j] /= pivot;

        // Eliminate other rows
        for(int k=0;k<m;k++)
        {
            if(k==i) continue;

            float factor = aug[k][i];
            for(int j=0;j<2*m;j++)
                aug[k][j] -= factor * aug[i][j];
        }
    }

    // Extract inverse
    for(int i=0;i<m;i++)
        for(int j=0;j<m;j++)
            A_inv[i][j] = aug[i][j+m];

    return true;
}
void matrixMultiply(int rowsA, int colsA, int colsB,
                    float *A,
                    float *B,
                    float *C)
{
    for (int i = 0; i < rowsA; i++) {
        for (int j = 0; j < colsB; j++) {

            C[i*colsB + j] = 0.0f;

            for (int k = 0; k < colsA; k++) {
                C[i*colsB + j] +=
                    A[i*colsA + k] *
                    B[k*colsB + j];
            }
        }
    }
}
void kalman_predict( float dt){
  // increment the heading estimate by the z axis portion of the gyroscope data
  x[0][0] += ((mpu_data[5])*GYRO_UNITS_TO_RADIANS-x[1][0])*dt;
  //speed estimate now constant velocity because of high accelerometer noise
  x[2][0] =x[2][0]*(1-speedometer_confidence)+((speedLeft + speedRight)/2)*speedometer_confidence;
  //predict location
  x[3][0] += x[2][0]*cos(x[0][0])*dt;
  x[4][0] += x[2][0]*sin(x[0][0])*dt;
  boundaries();
  // ---- Covariance prediction ----
  //jacobian F (F_transpose)
  float F[kalman_states][kalman_states] = {0};
  float F_transpose[kalman_states][kalman_states] = {0};
  for(int i=0;i<kalman_states;i++){
      F[i][i] = 1.0f;//diagonals are 1
      F_transpose[i][i] = 1.0f;//diagonals are 1
  }
  //nonlinear portion
  F[0][1] = -dt;//heading relates to gyro bias
      F_transpose[1][0]=F[0][1];
  F[3][2] =cos(x[0][0])*dt;//speed relates to location
      F_transpose[2][3]=F[3][2];
  F[4][2] =sin(x[0][0])*dt;//speed relates to location
      F_transpose[2][4]=F[4][2];
  float FP[kalman_states][kalman_states];
  //P=FPF_transpose
  matrixMultiply(kalman_states,kalman_states,kalman_states,(float*)F,(float*)P,(float*)FP);
  matrixMultiply(kalman_states,kalman_states,kalman_states,(float*)FP,(float*)F_transpose,(float*)P);
  
  for(int i=0;i<kalman_states;i++){
    P[i][i]+=Q[i]*dt;// add procces noise
  }
}
void kalman_update() {
  // ---- Measurement vector ----
  float z[kalman_measurements][1] = { 
    {gps_data[3]}, 
    {gps_data[2]},
    {gps_data[0]},
    {gps_data[1]}
  };

  // ---- Measurement matrix H ----
  float H[kalman_measurements][kalman_states] = {
    {1,0,0,0,0},
    {0,0,1,0,0},
    {0,0,0,1,0},
    {0,0,0,0,1},
  };
  //because this is used so often it was thought to store it in transposed form
  float H_transpose[kalman_states][kalman_measurements] = {
    {1,0,0,0},
    {0,0,0,0},
    {0,1,0,0},
    {0,0,1,0},
    {0,0,0,1}
  };
  // ---- Measurement noise ----
  float R[kalman_measurements][kalman_measurements] = {
    {R_gps_heading, 0, 0, 0},
    {0, R_gps_speed, 0, 0},
    {0, 0, R_gps_x, 0},
    {0, 0, 0, R_gps_y}
  };
  // ---- Innovation y = z - Hx ----
  float y[kalman_measurements];
  float Hx[kalman_measurements][1];
  matrixMultiply(kalman_measurements,kalman_states,1,(float *)H,(float *)x,(float *)Hx);
  for(int i=0;i<kalman_measurements;i++){
    y[i] = z[i][0]-Hx[i][0];
  }
  // Wrap heading innovation
  if(y[0] > PI) y[0] -= 2*PI;
  if(y[0] < -PI) y[0] += 2*PI;

  // ---- S = HPH^T + R ----
  float S[kalman_measurements][kalman_measurements] = {0};
  float PH_transpose[kalman_states][kalman_measurements];
  float HPH_transpose[kalman_measurements][kalman_measurements];
  matrixMultiply(kalman_states,kalman_measurements,kalman_states,(float *)P,(float *)H_transpose,(float *)PH_transpose);
  matrixMultiply(kalman_measurements,kalman_measurements,kalman_states,(float *)H,(float *)PH_transpose,(float *)HPH_transpose);
  for(int i=0;i<kalman_measurements;i++)
    for(int j=0;j<kalman_measurements;j++){
      S[i][j]=HPH_transpose[i][j]+R[i][j];
  }
  float S_inv[kalman_measurements][kalman_measurements];

  if(!mat_inverse(S, S_inv, kalman_measurements)) {
   // handle singular matrix (skip update)
    return;
  }
  // ---- K = P H^T S^-1 ----
  float K[kalman_states][kalman_measurements] = {0};
  matrixMultiply(kalman_states,kalman_measurements,kalman_measurements,(float *)PH_transpose,(float *)S_inv,(float *)K);

  // ---- x = x + K y ----
  for(int i=0;i<kalman_states;i++)
    for(int j=0;j<kalman_measurements;j++)
      x[i][0] += K[i][j] * y[j];

  // ---- Joseph Form Covariance Update ----
  // P = (I - K H) P (I - K H)^T + K R K^T

  float I[kalman_states][kalman_states] = {0};
  for(int i=0;i<kalman_states;i++) I[i][i] = 1;

  // KH = K * H
  float KH[kalman_states][kalman_states] = {0};
  matrixMultiply(kalman_states,kalman_measurements,kalman_states,(float *)K,(float *)H,(float *)KH);
  // A = I - KH
  float A[kalman_states][kalman_states];
  float A_transpose[kalman_states][kalman_states];
  for(int i=0;i<kalman_states;i++)
  for(int j=0;j<kalman_states;j++){
    A[i][j] = I[i][j] - KH[i][j];
    A_transpose[j][i]=I[i][j] - KH[i][j];
  }
  // AP = A*P
  float AP[kalman_states][kalman_states] = {0};
  matrixMultiply(kalman_states,kalman_states,kalman_states,(float *)A,(float *)P,(float *)AP);

  // newP = AP * A^T
  float newP[kalman_states][kalman_states] = {0};
  matrixMultiply(kalman_states,kalman_states,kalman_states,(float *)AP,(float *)A_transpose,(float *)newP);
  // KR = K * R
  float KR[kalman_states][kalman_measurements] = {0};
  matrixMultiply(kalman_states,kalman_measurements,kalman_measurements,(float *)K,(float *)R,(float *)KR);
  // KRK^T term
  for(int i=0;i<kalman_states;i++)
    for(int j=0;j<kalman_states;j++)
      for(int k=0;k<kalman_measurements;k++)
        newP[i][j] += KR[i][k] * K[j][k];  // K^T indexing

  //enforce symetry
  for(int i=0;i<kalman_states;i++)
    for(int j=i+1;j<kalman_states;j++){
      float avg = 0.5f * (newP[i][j] + newP[j][i]);
      newP[i][j] = avg;
      newP[j][i] = avg;
  }

  // Copy back
  for(int i=0;i<kalman_states;i++)
    for(int j=0;j<kalman_states;j++)
      P[i][j] = newP[i][j];
  boundaries();
}
void boundaries(){
  // Wrap heading
  while(x[0][0] > PI) x[0][0] -= 2*PI;
  while(x[0][0] < -PI) x[0][0] +=2*PI;
  //speed boundaries
  if(x[2][0]<0) x[2][0]=0;
  if(x[2][0]>max_speed_resonable) x[2][0]=max_speed_resonable;
}
void update_start(){//for when the system starts and doesn't know its own location and bias to be run every time the vehicle is stopped.
  start_latitude *=start_location_confidence;
  start_latitude  +=(gps.location.lat()*(1-start_location_confidence));
  start_longitude*=start_location_confidence;
  start_longitude+=(gps.location.lng()*(1-start_location_confidence));
  cosine_latitude = cos(radians(start_latitude));
  x[1][0]*=.9;
  x[1][0] += mpu_data[5] * 0.1; // gyro bias estimate
}
void loop() {
  mode = (mode & 0xFE) | (digitalRead(D5) & 0x01);
  drive();
  mpu();
  // ---- TIME STEP ----
  unsigned long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;
  //when time wraps around ignore one frame
  if(dt <= 0) return;
  if(mode&1){
  // ---- PREDICT ----
  kalman_predict(dt);
  }
  // ---- GPS UPDATE ----
  if(gps_update()) {// if the gps updated and the car is moving
    if((mode&1)==0){//if stopped use data to reset the function
    update_start();
    bluetooth(0,mode);
    bluetooth(4,0);
    return;
  } else if(gps_data[2] > minimum_speed_for_kalman_update){
    kalman_update();
    bluetooth(5,0);
    }
  }
  if(gps.satellites.value() < 4){
    bluetooth(3,gps.satellites.value());
  }
  bluetooth(6,0);
}