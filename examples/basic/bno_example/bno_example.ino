#include <VBCoreG4_arduino_system.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>



//Adafruit_BNO055 bno1;  
Adafruit_BNO055 bno2;  

sensors_event_t event1, event2;

void setup() {
  Serial.begin(115200);
 
  Wire.setSDA(PB_7_ALT1);
  Wire.setSCL(PC6);
  Wire.begin();
  bno2 = Adafruit_BNO055(56, 0x29, & Wire);
  //bno1 = Adafruit_BNO055(55, 0x28, & Wire);

  while (!Serial) delay(10);  // wait for serial port to open!
  if (!bno2.begin())
  {
    Serial.println("No BNO055 detected");
    while (1);
  }
  else{Serial.println("Smth detectes");}
  
}

float orientation_x; //
float orientation_y;
float orientation_z;
uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //how often to read data from the board

void loop() {
 // bno1.getEvent(&event1, Adafruit_BNO055::VECTOR_EULER);
  bno2.getEvent(&event2, Adafruit_BNO055::VECTOR_EULER);
  
  Serial.print("Курс: ");
  Serial.print(event2.orientation.x, 2);
  Serial.print(" | Тангаж: ");
  Serial.print(event2.orientation.y, 2);
  Serial.print(" | Крен: ");
  Serial.println(event2.orientation.z, 2);
  // pendulum_angle = orientationData.orientation.x;
  // pendulum_vel =angVelData.gyro.z;
}



// function constraining the angle in between -pi and pi, in degrees -180 and 180
float constrainAngle(float x){
    //Serial.print(x-180);
    x = x*_PI/180;
    return x - _PI;
}

