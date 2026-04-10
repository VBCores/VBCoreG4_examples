#include <VBCoreG4_arduino_system.h>

#define BTN PC3
/*
  
      Считывает цифровой вход c кнопки, выводит результат на монитор последовательного интерфейса.
      Кнопка подключена к цифровому выводу PC13.  В библиотеке VBCoreG4_arduino_system.h он определен как USR_BTN

    */

// процедура настройки запускается один раз при нажатии кнопки reset:
void setup() {
  // инициализировать последовательную коммуникацию на скорости 115200 бит в секунду:
  Serial.begin(115200);
  //назначить вывод кнопки входом:
  pinMode(BTN, INPUT_PULLUP);
}

void loop() {
  // считывание входного пина:
  int buttonState = digitalRead(BTN);
  // вывести состояние кнопки:
  Serial.println(buttonState);
  delay(10);     
}
