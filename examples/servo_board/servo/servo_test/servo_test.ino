#include <VBCoreG4_arduino_system.h>
#include <Servo.h>

#define SERVO_PIN PB2

Servo myServo;

void setup() {
  // Инициализация последовательного порта для отладки
  Serial.begin(115200);
 
  myServo.attach(SERVO_PIN);
  
  // Установка светодиода (из вашего примера) для индикации работы
  pinMode(LED2, OUTPUT);
}

void loop() {
  // Движение от 0 до 180 градусов
  Serial.println("Moving to 180 degrees");
  digitalWrite(LED2, HIGH); // Включаем LED при движении "туда"
  
  for (int pos = 0; pos <= 180; pos += 1) { 
    myServo.write(pos);
    delay(15); // Скорость движения
  }

  // Движение от 180 до 0 градусов
  Serial.println("Moving to 0 degrees");
  digitalWrite(LED2, LOW); // Выключаем LED при движении "обратно"
  
  for (int pos = 180; pos >= 0; pos -= 1) {
    myServo.write(pos);
    delay(15);
  }
}