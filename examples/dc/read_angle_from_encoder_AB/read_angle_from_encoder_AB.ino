#include <VBCoreG4_arduino_system.h>

#define Enc_A PB6  
#define Enc_B PC7
#define Enc_Z PA0

int count = 0;
int rotations = 0;

void ISR_A(){

  if (digitalRead(Enc_A) == digitalRead(Enc_B)){
    count -= 1;
  }
  else {
    count += 1;
  }

  if (count < -3600){
    rotations += 1;
    
    count = 0;
  }
  else if (count > 3600){
    rotations -= 1;

    count = 0;
  }
  

}

float get_angle_gradus() { // на 1 оборот 3600 импульсов, измерение угла от того положения в котором находился мотор при включении
  if (count > 0){
    return count*360/3600;
  }
  else{
    return (3600- abs(count))*360/3600;
  }
}

float get_angle_radian() { // на 1 оборот 3600 импульсов, измерение угла от того положения в котором находился мотор при включении
   if (count > 0){
    return count*2*M_PI/3600;
  }
  else{
    return (3600- abs(count))*2*M_PI/3600;
  }
}


void setup() {
  pinMode(LED1, OUTPUT);  //PD2
  pinMode(LED2, OUTPUT);  //PA5

  pinMode(Enc_B, INPUT);
  pinMode(Enc_Z, INPUT);
  attachInterrupt(digitalPinToInterrupt(Enc_A), ISR_A, CHANGE);

  Serial.begin(115200);

}

void loop() {

  Serial.print(count);
  Serial.print(" ");
  Serial.print(get_angle_gradus());
  Serial.print(" ");
  Serial.print(get_angle_radian());
  Serial.print(" ");
  Serial.println(rotations);


}
