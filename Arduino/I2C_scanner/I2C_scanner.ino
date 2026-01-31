#include <Wire.h>

void setup() {//Initialize
  // Start serial communication
  Serial.begin(115200);
  Serial.println("I2C Scanner");

  // Initialize I2C communication
  Wire1.begin();

  // Wait for the serial monitor to open
  delay(2000);

  // Scan the I2C bus for connected devices
  scanI2CDevices();}

void loop() {
  // Do nothing in loop
}

// Function to scan I2C devices
void scanI2CDevices() {
  byte error, address;
  int nDevices = 0;

  // Scan addresses from 1 to 127
  for (address = 1; address < 127; address++) {
    //Serial.println(address);
    // Begin transmission to the device
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0) {
      // Device found, print the address
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");  // Print address with leading zero
      }
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    } else if (error == 4) {
      // If there's a communication error, print it
      Serial.print("Unknown error at address 0x");
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.print(nDevices);
    Serial.println(" device(s) found");
  }
}
