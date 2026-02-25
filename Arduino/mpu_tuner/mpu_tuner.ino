// This code has 2 functions firs to verify the function of the device and second to calculate the standard offsetts of the device to account for bias
#include <Wire.h>

// MPU6050 constants
#define MPU6050_ADDR 0x69  // MPU6050 I2C address (default: 0x68)
#define PWR_MGMT_1   0x6B  // Power management register
#define MPU6050_BEGINING 0x3B  // Accelerometer data registers
#define ACCEL_CONFIG 0x1C  // Accelerometer configuration register
#define GYRO_CONFIG 0x1B   // Gyroscope configuration register
#define ACCEL_CONFIG_VALUE 0  // sets limits to 2g
#define GYRO_CONFIG_VALUE 0  // sets limits to 250°/s       
void setup() {
  // Initialize I2C communication
  Wire.begin();
  Serial.begin(115200);

  // Wake up the MPU6050 as it starts in sleep mode
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0);  // Set the PWR_MGMT_1 register to 0 to wake up the sensor
  Wire.endTransmission(true);

  // Configure the accelerometer and gyroscope
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(ACCEL_CONFIG);          // Select the accelerometer configuration register
  Wire.write(ACCEL_CONFIG_VALUE);    // Set the accelerometer to 2g
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(GYRO_CONFIG);           // Select the gyroscope configuration register
  Wire.write(GYRO_CONFIG_VALUE);     // Set the gyroscope to 250°/s
  Wire.endTransmission(true);

  // Wait for a brief moment to allow the sensor to settle
  delay(100);
}

void loop() {
  // Variables to store sensor data
  int16_t ax, ay, az, gx, gy, gz;
  int16_t temperature;
  long ax_sum = 0, ay_sum = 0, az_sum = 0, gx_sum = 0, gy_sum = 0, gz_sum = 0, temperature_sum=0;
  long num_samples = 100;  // Number of samples for averaging do not exceed 65536 or bit overflow might happen

  // Collect data for averaging
  for (long i = 0; i < num_samples; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_BEGINING);  // Start reading from the first accelerometer register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14);  // Request 14 bytes (6 for accelerometer, 6 for gyroscope, and 2 for temperature)

    // Read accelerometer data (3 axes, 2 bytes each)
    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();

    temperature =(Wire.read() << 8) | Wire.read();
    
    // Read gyroscope data (3 axes, 2 bytes each)
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();

    // Accumulate the values for averaging
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    temperature_sum+=temperature;
    delay(10);  // Small delay between samples
  }

  // Calculate the averages
  long ax_avg = ax_sum / num_samples;
  long ay_avg = ay_sum / num_samples;
  long az_avg = az_sum / num_samples;
  long gx_avg = gx_sum / num_samples;
  long gy_avg = gy_sum / num_samples;
  long gz_avg = gz_sum / num_samples;
  long temperature_avg=temperature_sum/num_samples;

  // Multiply by 10000 for each axis and print the results
  Serial.print("Accel X avg "); Serial.println(ax_avg );
  Serial.print("Accel Y avg "); Serial.println(ay_avg);
  Serial.print("Accel Z avg "); Serial.println(az_avg);
  Serial.print("Gyro X avg  "); Serial.println(gx_avg);
  Serial.print("Gyro Y avg: "); Serial.println(gy_avg );
  Serial.print("Gyro Z avg  "); Serial.println(gz_avg );
  Serial.print("temp avg  "); Serial.println(temperature_avg );
  delay(200);  // Delay for .2 seconds before the next read cycle
}
