#include <VBCoreG4_arduino_system.h>

#define Enc_A PB6  
#define Enc_B PC7


int count = 0;
int rotations = 0;
float t = 0;
float prev_count = 0;
float vel = 0;
HardwareTimer *timer = new HardwareTimer(TIM3);
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

void get_velocity(){
 vel = ((count - prev_count)*2*M_PI/3600)/0.01;
 prev_count = count;
}

void setup() {
  pinMode(LED1, OUTPUT);  //PD2
  pinMode(LED2, OUTPUT);  //PA5

  pinMode(Enc_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Enc_A), ISR_A, CHANGE);

  Serial.begin(115200);
  
  timer->pause();
  timer->setOverflow(100, HERTZ_FORMAT);
  timer->attachInterrupt(get_velocity);
  timer->refresh();
  timer->resume();
}

void loop() {

  Serial.println(vel);

}
