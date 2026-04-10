#include <VBCoreG4_arduino_system.h>

#define Enc_A PB6  
#define Enc_B PC7

#define IN1 PA8
#define IN2 PA9
#define SLEEPn PB3
#define VrefPin PA4
#define USR_BTN PC13


int pwm = 0;
int magnet = 22;
volatile long count = 0;
int prev_count = 0;
int rotations = 0;
int rotations_enc = 0;
int gear_ratio = 56;
int ipr = magnet* gear_ratio; //1240;

void ISR_A(){

  if (digitalRead(Enc_A) == digitalRead(Enc_B)){
    count += 1;
  }
  else {
    count -= 1;
  }

  // if (count < -ipr){ // 1240 импульсов на оборот
  //   rotations -= 1;
  //   count = 0;
  // }
  // else if (count > ipr){
  //   rotations += 1;
  //   count = 0;
  

}



void setup() {
  pinMode(LED1, OUTPUT);  //PD2
  pinMode(LED2, OUTPUT);  //PA5
  pinMode(Enc_A, INPUT);
  pinMode(Enc_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Enc_A), ISR_A, CHANGE);
  
  pinMode(PB3, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(VrefPin, OUTPUT);
  pinMode(USR_BTN, INPUT_PULLUP);
 
  analogWriteFrequency(25000);
  digitalWrite(SLEEPn, HIGH);
  digitalWrite(VrefPin, 2500);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  Serial.begin(115200);

}

void loop() {

  
  if (pwm>=0) {
      analogWrite(IN1, 255);
      analogWrite(IN2, 255-pwm);
    }
  else {
    analogWrite(IN2, 255);
    analogWrite(IN1, 255-abs(pwm)); 
  }
  
  if (Serial.available() > 0) {
    pwm = Serial.readString().toInt();    
  }
  // Serial.print(digitalRead(Enc_A));
  // Serial.print(" ");
  // Serial.print(digitalRead(Enc_B));
  // Serial.print(" ");
  
  Serial.println(count);


}
