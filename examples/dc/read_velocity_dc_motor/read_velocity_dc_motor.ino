#include <VBCoreG4_arduino_system.h>

#define Enc_A PB6  
#define Enc_B PC7

#define IN1 PA8
#define IN2 PA9
#define SLEEPn PB3
#define USR_BTN PC13

int forward = HIGH;
int prev_btnSt;
int buttonState;
int is_stopped = 1;

int count = 0;
int rotations = 0;
float t = 0;
float prev_count = 0;
float vel = 0;
int ipr = 1240;
HardwareTimer *timer = new HardwareTimer(TIM3);


void ISR_A(){

  if (digitalRead(Enc_A) == digitalRead(Enc_B)){
    count -= 1;
  }
  else {
    count += 1;
  }

  if (count < -ipr){ // 1240 импульсов на оборот
    rotations += 1;
    count = 0;
  }
  else if (count > ipr){
    rotations -= 1;
    count = 0;
  }
  

}

void get_velocity(){
 vel = ((count - prev_count)*2*M_PI/ipr)/0.04;
 prev_count = count;
}

void setup() {
  pinMode(LED1, OUTPUT);  //PD2
  pinMode(LED2, OUTPUT);  //PA5
  pinMode(Enc_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Enc_A), ISR_A, CHANGE);
  
  pinMode(PB3, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(USR_BTN, INPUT_PULLUP);
 
  digitalWrite(SLEEPn, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  prev_btnSt = digitalRead(USR_BTN);

  Serial.begin(115200);

  timer->pause();
  timer->setOverflow(25, HERTZ_FORMAT);
  timer->attachInterrupt(get_velocity);
  timer->refresh();
  timer->resume();

}

void loop() {

  buttonState = digitalRead(USR_BTN);
  if(buttonState == LOW && prev_btnSt == HIGH){
    forward = !forward;
    prev_btnSt = buttonState;
  }
  if(buttonState == HIGH) {prev_btnSt = buttonState;}
  
  if (forward == HIGH && is_stopped == LOW){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  }

  else if (forward == LOW && is_stopped == LOW){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  }

  else{
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
  }
  
  if (Serial.available() > 0) {
    is_stopped = Serial.readString().toInt();
    
  }
 
  // Serial.print(count);
  // Serial.print(" ");
  Serial.print(rotations);
  Serial.print(" ");
  Serial.println(vel); // rad/sec
  // Serial.print(" ");
  // Serial.println(vel); // rad/sec



}
