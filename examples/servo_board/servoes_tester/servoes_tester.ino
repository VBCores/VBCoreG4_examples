#include <VBCoreG4_arduino_system.h>
#include <Servo.h>

// Определение пинов согласно вашему списку
#define SERVO_PIN_1 PA1
#define SERVO_PIN_2 PA0
#define SERVO_PIN_3 PA4
#define SERVO_PIN_4 PA6
#define SERVO_PIN_5 PB0
#define SERVO_PIN_6 PB1
#define SERVO_PIN_7 PB2
#define SERVO_PIN_8 PB14

// Создание массива объектов Servo для удобного управления в цикле
Servo myServos[8];
int servoPins[] = {SERVO_PIN_1, SERVO_PIN_2, SERVO_PIN_3, SERVO_PIN_4, SERVO_PIN_5, SERVO_PIN_6, SERVO_PIN_7, SERVO_PIN_8};

void setup() {
  // Инициализация последовательного порта для отладки
  Serial.begin(115200);
  Serial.println("Servo Tester Initializing...");

  // Привязка каждого сервопривода к соответствующему пину
  for (int i = 0; i < 8; i++) {
    myServos[i].attach(servoPins[i]);
  }
  
  // Установка светодиода (из вашего примера) для индикации работы
  pinMode(LED2, OUTPUT);
}

void loop() {
  // Движение от 0 до 180 градусов
  Serial.println("Moving to 180 degrees");
  digitalWrite(LED2, HIGH); // Включаем LED при движении "туда"
  
  for (int pos = 0; pos <= 180; pos += 1) { 
    for (int i = 0; i < 8; i++) {
      myServos[i].write(pos);
    }
    delay(15); // Скорость движения
  }

  // Движение от 180 до 0 градусов
  Serial.println("Moving to 0 degrees");
  digitalWrite(LED2, LOW); // Выключаем LED при движении "обратно"
  
  for (int pos = 180; pos >= 0; pos -= 1) {
    for (int i = 0; i < 8; i++) {
      myServos[i].write(pos);
    }
    delay(15);
  }
}