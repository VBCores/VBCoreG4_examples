//пример для DC драйвера, чтение дип переключателей, и запись их в ID
// если переключатели стоят в положении 0100, то ID = 4, для 0011 ID = 3
#include <VBCoreG4_arduino_system.h>

#define DIP_1 PB10
#define DIP_2 PB11
#define DIP_3 PB12
#define DIP_4 PB13

int ID;
void setup() {
  Serial.begin(115200);
  pinMode(DIP_1, INPUT_PULLDOWN);
  pinMode(DIP_2, INPUT_PULLDOWN);
  pinMode(DIP_3, INPUT_PULLDOWN);
  pinMode(DIP_4, INPUT_PULLDOWN);

  uint8_t mask = digitalRead(DIP_1);
  mask = (mask<<1)^digitalRead(DIP_2);
  mask = (mask<<1)^digitalRead(DIP_3);
  mask = (mask<<1)^digitalRead(DIP_4);

  Serial.println(mask);
  
  ID = (int)mask;
  Serial.println(ID);
}

void loop() {
  
}
