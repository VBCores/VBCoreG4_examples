#include <VBCoreG4_arduino_system.h>

const float ampl = 1.0; // амплитуда синусоидального сигнала
const float freq = 0.2; // частота синусоидального сигнала
float t = 0; 
float out = 0;
float curve = 0;

void setup() {
  pinMode(PA5, OUTPUT); // инициализировать пин PA5 в режим выхода
  analogWriteResolution(12); // задать разрешение ЦАП 

  t = 0;
}

void loop() {
  // инкремент 
  t += 0.001;
  // расчет синусоидального сигнала от 0 до 1
  curve = (ampl*sin(2*M_PI*freq*t) + ampl)/2;
  // перевод в единицы ЦАП
  out = curve * 4095;
  // вывод полученного значения на ЦАП
  analogWrite(PA5, out);
  // задержка 1 мс
  delay(1);
}