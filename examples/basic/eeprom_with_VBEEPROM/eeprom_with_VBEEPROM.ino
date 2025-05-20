#include <VBCoreG4_arduino_system.h>
#include <Wire.h>
#include <VB_EEPROM.h>

#define EEPROM_DATA_ADDR 0x05

void setup() {
  Serial.begin(115200);
  initEEPROM();
  Serial.println("inited");
  clearEEPROM();
  Serial.println("cleared");
  float default_value = -5.8;
  if (!isDataInEEPROM(EEPROM_DATA_ADDR)) {
    Serial.println("EEPROM пуста. Записываю значение по умолчанию...");
    writeFloatToEEPROM(EEPROM_DATA_ADDR, default_value);
  } else {
    Serial.println("В EEPROM уже есть значение.");
  }
  delay(5);
  float data = readFloatFromEEPROM(EEPROM_DATA_ADDR);
  Serial.println(data);
}

void loop() {
  
}
