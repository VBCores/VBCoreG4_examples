//пример для BLDC драйвера, чтение дип переключателей, и запись их в EEPROM
// если дип переключатели стоят в положении 0100, то ID = 4, для 0011 ID = 3
#include <VBCoreG4_arduino_system.h>
#include <Wire.h>
#include <EEPROM.h>

#define EEPROM_I2C_ADDR 0x50  

#define DIP_1 PB2
#define DIP_2 PB10
#define DIP_3 PB11
#define DIP_4 PB12

int ID;
uint8_t mask;
void setup() {
  Serial.begin(115200);
  Wire.setSDA(pinSDA); //PB_7_ALT1
  Wire.setSCL(pinSCL); //PC6
  Wire.begin();

  //EEPROM.put(0x01, 0xFF); // очистить eeprom
  byte val = EEPROM.read(0x01); // можно использовать от 0x0000 до 0x7FFF
  Serial.print("Прочитано значение: 0x");
  Serial.println(val, HEX);

  pinMode(DIP_1, INPUT_PULLDOWN);
  pinMode(DIP_2, INPUT_PULLDOWN);
  pinMode(DIP_3, INPUT_PULLDOWN);
  pinMode(DIP_4, INPUT_PULLDOWN);

  if (val == 0xFF){
    mask = digitalRead(DIP_1);
    mask = (mask<<1)^digitalRead(DIP_2);
    mask = (mask<<1)^digitalRead(DIP_3);
    mask = (mask<<1)^digitalRead(DIP_4);
    ID = (int)mask;
    EEPROM.put(0x01, ID);   
  }
  else {ID = (int)val;}
  Serial.print("ID = ");
  Serial.println(ID);
}

void loop() {
  
}
