#include <VBCoreG4_arduino_system.h>

#define IN1 PA8
#define IN2 PA9
#define SLEEPn PB3
#define USR_BTN PC13
#define VREF PA4

#define Enc_A PB6  
#define Enc_B PC7
#define AIOUT PC1

int forward = HIGH;
int prev_btnSt;
int buttonState;
int duty = 0;
HardwareTimer *timer_move = new HardwareTimer(TIM3);

int count = 0;
int rotations = 0;
float t = 0;
float prev_count = 0;
float vel = 0;
int ipr = 1240;
HardwareTimer *timer_vel = new HardwareTimer(TIM5);

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
  analogReadResolution(12);
  pinMode(Enc_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Enc_A), ISR_A, CHANGE);
  pinMode(AIOUT, INPUT);
 
  pinMode(PB3, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(USR_BTN, INPUT_PULLUP);
  pinMode(VREF, OUTPUT);
 
  digitalWrite(SLEEPn, HIGH);
  digitalWrite(VREF, HIGH);
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  prev_btnSt = digitalRead(USR_BTN);

  Serial.begin(500000);

  timer_move->pause();
  timer_move->setOverflow(100, HERTZ_FORMAT); 
  timer_move->attachInterrupt(move);
  timer_move->refresh();
  timer_move->resume();

  timer_vel->pause();
  timer_vel->setOverflow(25, HERTZ_FORMAT);
  timer_vel->attachInterrupt(get_velocity);
  timer_vel->refresh();
  timer_vel->resume();

  delay(1000);
}
float v_sense;
void loop() {
  buttonState = digitalRead(USR_BTN);
  if(buttonState == LOW && prev_btnSt == HIGH){
    forward = !forward;
    prev_btnSt = buttonState;
    analogWrite(IN1, 255);
    analogWrite(IN2, 255);
    delay(1);
  }
  if(buttonState == HIGH) {prev_btnSt = buttonState;}
  if (Serial.available() > 0) {
    duty = Serial.readString().toInt();
  }
  v_sense = 10*(analogRead(AIOUT)*3.3/4096);
  // Serial.print(v_sense);
  // Serial.print(" ");
  Serial.println(vel);
}

void move(){
  if (forward == HIGH) {
      analogWrite(IN1, 255-duty);
    }
  if (forward == LOW) {
    analogWrite(IN2, 255-duty);
  }
}

