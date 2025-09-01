/*В этом примере прграмма считывает через сериал монитор с какой частотой мигать светодиодом. 
Частота задается в герцах. Нажатием пользовательской кнопки светодиод можно отключить вообще. 
Повторное нажатике пользовательской кнопки включает моргание светодиода */
#include <VBCoreG4_arduino_system.h>

#define LED2 PA5
#define USR_BTN PC13

double frec = 1;

int is_off = LOW;
int prev_btnSt;
int buttonState;
HardwareTimer *timer = new HardwareTimer(TIM3);
void setup() {
  Serial.begin(115200);
  pinMode(USR_BTN, INPUT_PULLUP);
  pinMode(LED2, OUTPUT);
  prev_btnSt = digitalRead(USR_BTN);

  timer->pause();
  timer->setOverflow(1, HERTZ_FORMAT); 
  timer->attachInterrupt(blink);
  timer->refresh();
  timer->resume();
}

void loop() { 
  buttonState = digitalRead(USR_BTN);
  if(buttonState == LOW && prev_btnSt == HIGH){
    is_off = !is_off;
    prev_btnSt = buttonState;
  }
  if(buttonState == HIGH) {prev_btnSt = buttonState;}
  if (Serial.available() > 0) {
    frec = Serial.readString().toDouble();
    if(frec>=1){timer->setOverflow(frec, HERTZ_FORMAT);}
    else if (frec> 0 && frec<1){timer->setOverflow(1000000/frec, MICROSEC_FORMAT);}
    else {Serial.println("0 or negative frequency");}
    timer->refresh();
    timer->resume();
  }

}

void blink(){
  if(is_off == LOW){
    digitalWrite(LED2, !digitalRead(LED2));  
  }
  else{digitalWrite(LED2, LOW);}
}

