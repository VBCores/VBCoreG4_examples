#include <VBCoreG4_arduino_system.h>

#define IN1 PA8
#define IN2 PA9
#define SLEEPn PB3
#define USR_BTN PC13
#define VREF PA4

/*Пример вращения dc мотором
Напряжение, подаваемое мотором регулируется ШИМ сигналом
ШИМ задается от 0 до 255 по сериал порту
*/
int pwm = 0;
HardwareTimer *timer = new HardwareTimer(TIM3);

void setup() {
 
  pinMode(PB3, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(USR_BTN, INPUT_PULLUP);
  pinMode(VREF, OUTPUT);
 
  digitalWrite(SLEEPn, HIGH);
  digitalWrite(VREF, HIGH);
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);

  Serial.begin(115200);

  timer->pause();
  timer->setOverflow(1000, HERTZ_FORMAT); 
  timer->attachInterrupt(move);
  timer->refresh();
  timer->resume();

  delay(1000);
}

void loop() {
  if (Serial.available() > 0) {
    pwm = Serial.readString().toInt();
  }
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

