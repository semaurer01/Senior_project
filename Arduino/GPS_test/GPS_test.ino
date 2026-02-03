#include "SparkFun_I2C_GPS_Arduino_Library.h" // https://github.com/sparkfun/SparkFun_I2C_GPS_Arduino_Library
#include "TinyGPSPlus.h"

I2CGPS myI2CGPS; // Hook object to the library
TinyGPSPlus gps;
time_t last_update=0;
void setup() {
  Serial.begin(115200);
  Serial.println("XA1110 Robust I2C GPS Example");

  // Initialize GPS over Wire1 (Qwiic/I2C)
  while (!myI2CGPS.begin(Wire1)) {
    Serial.println("Module failed to respond. Check wiring!");
    delay(500);
  }
  Wire1.setClock(400000); // 400 kHz

  Serial.println("GPS module found!");
    // Enable debugging output
  myI2CGPS.enableDebugging(Serial);
    // Build the PMTK packet for 10 Hz (100 ms)
  String config = myI2CGPS.createMTKpacket(220, ",100"); 
  // Send it
  myI2CGPS.sendMTKpacket(config);
}

void loop() {
  // Read all available bytes from GPS
    int bytesAvailable = myI2CGPS.available();

  for (int i = 0; i < bytesAvailable; i++) {
    char c = myI2CGPS.read();
    gps.encode(c);
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
    last_update=millis();
  }else if(millis()>last_update+1000){
    Serial.print("no data Sats: ");
    Serial.println(gps.satellites.value());
    last_update=millis();
  }
}