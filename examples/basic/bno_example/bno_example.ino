#include <VBCoreG4_arduino_system.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>



Adafruit_BNO055 bno28;  
Adafruit_BNO055 bno29;  

sensors_event_t event28, event29;

void setup() {
  Serial.begin(115200);
  pinMode(PB0, OUTPUT);

  digitalWrite(PB0, LOW);
  delay(1000);
  digitalWrite(PB0, HIGH);
  delay(1000);

  Wire.setSDA(PB_7_ALT1);
  Wire.setSCL(PC6);
  Wire.begin();
  bno28 = Adafruit_BNO055(55, 0x28, & Wire);
  bno29 = Adafruit_BNO055(56, 0x29, & Wire);
  

  while (!Serial) delay(10);  // wait for serial port to open!
  if (!bno28.begin() )
  {
    Serial.print("No 28 BNO055 detected");
    while (1);
  }
  if (!bno29.begin() )
  {
    Serial.print("No 29 BNO055 detected");
    while (1);
  }
    
}

void loop() {
 bno28.getEvent(&event28, Adafruit_BNO055::VECTOR_LINEARACCEL);
 bno29.getEvent(&event29, Adafruit_BNO055::VECTOR_LINEARACCEL);
 // bno29.getEvent(&event29, Adafruit_BNO055::VECTOR_EULER);
 // bno28.getEvent(&event28, Adafruit_BNO055::VECTOR_EULER);
 
  Serial.print(event28.acceleration.y);
  Serial.print(" , ");
  Serial.println(event29.acceleration.y+20);

  delay(100);

}

