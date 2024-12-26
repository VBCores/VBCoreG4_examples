#include <VBCoreG4_arduino_system.h>

#define Enc_A PB6  
#define Enc_B PC7
#define IN1 PA8
#define IN2 PA9
#define SLEEPn PB3
#define VREF PA4

/*Пример чтения энкодера и получения с него угла и скорости мотора
В одной функции угол считается от 0 до 6.28 радиан 
В другой - от 0 до 360 градусов
В сериал выводится значения угла в радианах и угловой скорости в радианах в секунду
Пример показывает как читать данные с энкодера
Угловая скорость считается как разность углов деленая на время между фиксацией значений этих углов
При обнулении новом обороте (когда после 6.28 радиан угол снова становится 0) угловая скорость считается некорректно
Чтобы этого избежать, значение угла должно накапливаться (6.28, 6.29, 6.3... ) это несложно дописать, добавив пару арифметических операций
*/
int count = 0;
int rotations = 0;

float t = 0;
float prev_count = 0;
float vel = 0;

int ipr = 1240;
HardwareTimer *timer = new HardwareTimer(TIM3);
void ISR_A(){

  if (digitalRead(Enc_A) == digitalRead(Enc_B)){
    count += 1;
  }
  else {
    count -= 1;
  }

  if (count < -ipr){
    rotations -= 1;
    
    count = 0;
  }
  else if (count > ipr){
    rotations += 1;

    count = 0;
  }
  

}

float get_angle_gradus() { 
  if (count > 0){
    return count*360/ipr;
  }
  else{
    return (ipr- abs(count))*360/ipr;
  }
}

float get_angle_radian() { 
   if (count > 0){
    return count*2*M_PI/ipr;
  }
  else{
    return (ipr- abs(count))*2*M_PI/ipr;
  }
}

void get_velocity(){
 vel = ((count - prev_count)*2*M_PI/ipr)/(millis()-t)*1000;
 t = millis();
 prev_count = count;
}

void setup() {
  pinMode(Enc_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Enc_A), ISR_A, CHANGE);

  pinMode(PB3, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(VREF, OUTPUT);
 
  digitalWrite(SLEEPn, HIGH);
  digitalWrite(VREF, HIGH);
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  Serial.begin(115200);
  
  timer->pause();
  timer->setOverflow(100, HERTZ_FORMAT);
  timer->attachInterrupt(get_velocity);
  timer->refresh();
  timer->resume();

  t = millis();
}

String msg;
int pwm = 0;

void loop() {
  msg = String("angle: ")+get_angle_radian()+" vel: "+vel;
  Serial.println(msg);
  if (Serial.available() > 0) {
    pwm = Serial.readString().toInt();
  } 
  move();

}

void move(){
  if (pwm>=0) {
      analogWrite(IN1, 255);
      analogWrite(IN2, 255-pwm);
    }
  else {
    analogWrite(IN2, 255);
    analogWrite(IN1, 255-abs(pwm)); 
  }
}
