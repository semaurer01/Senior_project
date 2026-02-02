#include "SparkFun_I2C_GPS_Arduino_Library.h" // https://github.com/sparkfun/SparkFun_I2C_GPS_Arduino_Library
#include "TinyGPSPlus.h"

I2CGPS myI2CGPS; // Hook object to the library
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);
  Serial.println("XA1110 Robust I2C GPS Example");

  // Initialize GPS over Wire1 (Qwiic/I2C)
  while (!myI2CGPS.begin(Wire1)) {
    Serial.println("Module failed to respond. Check wiring!");
    delay(500);
  }
  Serial.println("GPS module found!");
}

void loop() {
  // Read all available bytes from GPS
  while (myI2CGPS.available()) {
    char incoming = (char)myI2CGPS.read(); // Cast to char

    // Filter non-printable bytes
      Serial.print((int)incoming);
      Serial.print(" ");

    // Feed TinyGPSPlus parser
    gps.encode(incoming);
  }

  // Only print updated location when new data is available
  if (gps.location.isUpdated()) {
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print("  Lon: ");
    Serial.print(gps.location.lng(), 6);

    Serial.print("  Sats: ");
    Serial.print(gps.satellites.value());

    Serial.print("  HDOP: ");
    Serial.print(gps.hdop.hdop());

    Serial.println();
  }
}