#include <VBCoreG4_arduino_system.h>

#define M1_IN1 PB6
#define M1_IN2 PB4
#define M2_IN1 PB7
#define M2_IN2 PB5

int pwm1 = 255;
int pwm2 = 0;
HardwareTimer *timer = new HardwareTimer(TIM3);

void setup() {
 
 
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  analogWrite(M1_IN1, 0);
  analogWrite(M1_IN2, 0);
  analogWrite(M2_IN1, 0);
  analogWrite(M2_IN2, 0);

  Serial.begin(115200);
  analogWriteResolution(8);
  analogWriteFrequency(100);
  timer->pause();
  timer->setOverflow(1000, HERTZ_FORMAT); 
  timer->attachInterrupt(move);
  timer->refresh();
  timer->resume();

  delay(1000);
}

String str;
void loop() {
  if (Serial.available() > 0) {
    str = Serial.readStringUntil('\n');
    int index = str.indexOf(' ');
    pwm1 = str.substring(0, index).toInt();
    pwm2 = str.substring(index+1).toInt();
  }
  Serial.print(pwm1);
  Serial.print(" ");
  Serial.println(pwm2);
}

void move(){
  analogWrite(M1_IN1, pwm1);
  analogWrite(M1_IN2, pwm2);
  analogWrite(M2_IN1, pwm1);
  analogWrite(M2_IN2, pwm2);
  // if (pwm>=0) {
  //     analogWrite(IN1, 255);
  //     analogWrite(IN2, 255-pwm);
  //   }
  // else {
  //   analogWrite(IN2, 255);
  //   analogWrite(IN1, 255-abs(pwm)); 
  // }
}

