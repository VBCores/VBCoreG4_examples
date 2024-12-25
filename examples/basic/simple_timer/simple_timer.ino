#include <VBCoreG4_arduino_system.h>

// Пример работы с таймером и stm32duino, 
// по мотивам https://github.com/stm32duino/Arduino_Core_STM32/wiki/HardwareTimer-library
// В этом примере таймер настроен на 1 герц, для платы VBCore на STM32G746RE 
// Светодиод мигает по таймеру 1 раз в секунду



void setup() {
  //LED1 привязан к пину PD2 в библиотеке VBCoreG4_arduino_system
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);

  HardwareTimer *timer = new HardwareTimer(TIM3);
  timer->pause(); // останавливаем таймер перед настройкой
  timer->setOverflow(1, HERTZ_FORMAT); // 1 Hz 
  timer->attachInterrupt(func_timer); // активируем прерывание
  timer->refresh(); // обнулить таймер 
  timer->resume(); // запускаем таймер

}

void loop() {
  // put your main code here, to run repeatedly:
}

void func_timer() // обработчик прерывания
{
  
  digitalWrite(LED1, !digitalRead(LED1));
 
} 
