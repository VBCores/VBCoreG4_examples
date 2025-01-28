#include <VBCoreG4_arduino_system.h>

#define Enc_A PB6  
#define Enc_B PC7
#define IN1 PA8
#define IN2 PA9
#define SLEEPn PB3
#define VREF PA4

int count = 0;
int rotations = 0;
float pos;
float current_pos = 0;

int ipr = 1240;
HardwareTimer *timer_move = new HardwareTimer(TIM3);


void ISR_A(){

  if (digitalRead(Enc_A) == digitalRead(Enc_B)){
    count -= 1;
    Serial.println("Уменьшается");
  }
  else {
    count += 1;
    Serial.println("Увеличивается");
  }

  if (count < -ipr){
    rotations += 1;
    count = 0;
  }
  else if (count > ipr){
    rotations -= 1;
    count = 0;
  }
  

}

float get_angle_gradus() { // на 1 оборот 3600 импульсов, измерение угла от того положения в котором находился мотор при включении
  if (count > 0){
    return count*360/ipr;
  }
  else{
    return (ipr- abs(count))*360/ipr;
  }
}

float get_angle_radian() { // на 1 оборот 3600 импульсов, измерение угла от того положения в котором находился мотор при включении
   if (count > 0){

    current_pos = count*2*M_PI/ipr;
  }
  else{
    current_pos = (ipr- abs(count))*2*M_PI/ipr;
  }
}


void setup() {
  
  pinMode(Enc_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Enc_A), ISR_A, CHANGE);

  pinMode(VREF, OUTPUT);
 
  
  digitalWrite(VREF, HIGH);
  
  pinMode(SLEEPn, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  
 
  digitalWrite(SLEEPn, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);


  Serial.begin(115200);
  //get_angle_radian();
  pos = current_pos;
  Serial.println(current_pos);

  delay(1000);

}

void loop() {
  
  if (Serial.available() > 0) {
    pos =Serial.readString().toFloat();
    Serial.println(pos);
  }

  if (current_pos == pos) {
    digitalWrite (IN1, 0);
    digitalWrite (IN2, 0);
  }
  else if (current_pos > pos){
    digitalWrite (IN1, 1);
    digitalWrite (IN2, 0);
  }
  else {
    digitalWrite (IN1, 0);
    digitalWrite (IN2, 1);
  }
delay(100);
}
