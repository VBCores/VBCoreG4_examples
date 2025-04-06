#include <VBCoreG4_arduino_system.h>

HardwareTimer *timer = new HardwareTimer(TIM3);
const uint16_t numPoints = 100;
uint16_t sineTable[numPoints];
uint16_t i = 0;
void setup() {
  //Serial.begin(115200);
  pinMode(PA4, OUTPUT); // PA4 - DAC pin
  for (i = 0; i<100; ++i){
    float angle = (2.0f * PI * i) / 100;
    sineTable[i] = (uint16_t)((sin(angle) + 1.0f) * 2047.5f);
  }
  i = 0;
  timer->pause(); // останавливаем таймер перед настройкой
  timer->setOverflow(150, HERTZ_FORMAT);  
  timer->attachInterrupt(func_dac); // активируем прерывание
  timer->refresh(); // обнулить таймер 
  timer->resume(); // запускаем таймер
}

void loop() {
  // put your main code here, to run repeatedly:
}
void func_dac() // обработчик прерывания
{  
  analogWrite(PA4, sineTable[i]);
  //Serial.println(sineTable[i]);
  ++i;
  if (i>=numPoints) i = 0;
} 