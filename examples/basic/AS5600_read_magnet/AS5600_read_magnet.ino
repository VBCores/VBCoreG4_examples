#include <VBCoreG4_arduino_system.h>
#include <Wire.h>
#include <AS5600.h>

#define pinSDA PB_7_ALT1
#define pinSCL PC6

AMS_5600 sensor;

void setup() {
  Wire.setSDA(pinSDA);
  Wire.setSCL(pinSCL);
  Wire.begin();

  pinMode(LED2, OUTPUT);
  Serial.begin(500000);

  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>> ");
  if(sensor.detectMagnet() == 0 ){
    while(1){
        if(sensor.detectMagnet() == 1 ){
            Serial.print("Current Magnitude: ");
            Serial.println(sensor.getMagnitude());
            break;
        }
        else{
            Serial.println("Can not detect magnet");
        }
        delay(1000);
    }
  }
 
  
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(millis());
  Serial.print(";");
  Serial.println(sensor.getRawAngle());
  // digitalWrite(LED2, !digitalRead(LED2));
  // delay(1000);
}
