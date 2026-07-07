/*TT Motor
GM37-3530-1240-90
DC/12V 44RPM 220909
количество магнитов - 22*/
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
int count = 0;
int prev_count = 0;
int rotations = 0;
int rotations_enc = 0;
int gear_ratio = 90;
int ipr = magnet* gear_ratio;

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

void setup() {
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
  analogWriteResolution(12);
  analogWrite(VrefPin, 2500); // см в документации как считать Vref https://docs.vbcores.ru/docs/Examples/arduino-examples
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  Serial.begin(115200);

}

void loop() {

  
  if (pwm>=0) {
      analogWrite(IN1, 4095);
      analogWrite(IN2, 4095-pwm);
    }
  else {
    analogWrite(IN2, 4095);
    analogWrite(IN1, 4095-abs(pwm)); 
  }
  
  if (Serial.available() > 0) {
    pwm = Serial.readString().toInt();    
  }
 
  Serial.print(count);
  Serial.print(" ");
  Serial.println(rotations);

}
