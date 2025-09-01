// --------------------------------------
// i2c_scanner

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial); 
  Serial.println("\nI2C Scanner");

  pinMode(PB0, OUTPUT);

  digitalWrite(PB0, LOW);
  delay(1000);
  digitalWrite(PB0, HIGH);
  delay(1000);
  
  Wire.setSDA(PB_7_ALT1);
  Wire.setSCL(PC6);
  Wire.begin();
 

}

void loop() {
  int nDevices = 0;

  Serial.println("Scanning...");

  for (byte address = 1; address < 127; ++address) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");

      ++nDevices;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  } else {
    Serial.println("done\n");
  }
  delay(5000); // Wait 5 seconds for next scan
}
