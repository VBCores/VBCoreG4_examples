#include <VBCoreG4_arduino_system.h>

//в VBCoreG4_arduino_system.h пин PA5 определен как LED2 
// функция настройки запускается один раз, когда вы нажимаете сброс или подаете питание на плату

void setup() {
  // инициализировать цифровой вывод LED2 в качестве выхода.
  pinMode(LED2, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED2, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                    // wait for a second
  digitalWrite(LED2, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}
