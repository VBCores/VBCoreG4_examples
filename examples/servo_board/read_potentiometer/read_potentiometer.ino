#include <VBCoreG4_arduino_system.h>

#define AnalogPin PC0

int val = 0;


void setup() {
  // инициализировать цифровой вывод A0 в качестве аналогового входа
  pinMode(AnalogPin, INPUT_ANALOG);
  // инициализировать пин PA9 в режим выхода
  pinMode(LED2, OUTPUT); 
  // установить разрядность АЦП в 12 бит
  analogReadResolution(12);
  // задать разрядность ШИМ 
  analogWriteResolution(12); 
  // задать частоту ШИМ
  analogWriteFrequency(100); 
}

void loop() {
  // чтение данных с аналогового входа
  val = analogRead(AnalogPin);

  // вывод полученного значения на ЦАП
  analogWrite(LED2, val);
}