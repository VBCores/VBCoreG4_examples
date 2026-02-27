#include <VBCoreG4_arduino_system.h>


//в VBCoreG4_arduino_system.h пин PA5 определен как LED2 

//запустить can c распберри - sudo ip link set can0 up txqueuelen 65535 type can bitrate 1000000 dbitrate 8000000 fd on
//отправить сообщение c распберри - cansend can0 00000123#DEADBEEF, ID всегда содержит 8 цифр
//прочитать все сообщения в can -  candump can0
//прочитать сообщение can по его ID -  candump can0,ID:7ff, например, чтобы почитать сообщения с ID = 0x12: candump can0,0x12:7ff

//в libraries добавить библиотеку VBCoreG4_arduino_system
//функция init() запускает can
//функция get_hfdcan() возвращает переменную типа FDCAN_HandleTypeDef, без которой невозможно взаимодействие с can



uint8_t data[4] = { 222, 173, 190, 239}; //DE AD BE EF - сообщение, которое будем отправлять
unsigned long t;
FDCAN_HandleTypeDef*  hfdcan1; // создаем переменную типа FDCAN_HandleTypeDef
CanFD* canfd; // Класс CanFD - это управляющая  структура
FDCAN_TxHeaderTypeDef TxHeader; // хидер сообщения, которое будет отправляться
void setup() {
  Serial.begin(115200);
  pinMode(LED2, OUTPUT);
  
  /* Настройка FD CAN - обычно стандартная, ничего менять не нужно */
  SystemClock_Config();  // Настройка тактирования
  canfd = new CanFD();  // Создаем управляющий класс
  canfd->init(); // Инициализация CAN
  canfd->write_default_params();  // Записываем дефолтные параметры для FDCAN (1000000 nominal / 8000000 data)
  canfd->apply_config();  // Применяем их
  hfdcan1 = canfd->get_hfdcan();  // Сохраняем конфиг
  canfd->default_start(); // Стартуем can
 
  /*Заполняем хидер сообщения - указываем его ID (его можно менять), длину сообщения, тип ID - по умолчанию лучше использовать расширенный */
  TxHeader.Identifier = 0x12; // ID сообщения
  TxHeader.DataLength = FDCAN_DLC_BYTES_4; // Длина сообщение 4 байта
  TxHeader.IdType = FDCAN_EXTENDED_ID; // Всегда по умолчанию используем расширенный тип ID

}


void loop() {
  //------Отправка сообщения в can------
  if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan1) != 0){ // Если очередь сообщений свободна
    //Отправим сообщение
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &TxHeader, data) != HAL_OK){ Error_Handler(); } 
    else{digitalWrite(LED2, !digitalRead(LED2));} //помигаем светодиодом, если все отправлено
  }

  // -------Получение сообщений из can-------

  while(HAL_FDCAN_GetRxFifoFillLevel(hfdcan1, FDCAN_RX_FIFO0) > 0 ) //Пока в очереди есть получаемые сообщения
    {
      FDCAN_RxHeaderTypeDef Header;  // хидер для входящего сообщения
      uint8_t RxData[4]; // длина входящего сообщения - 4 байта, вообще максимальная длина сообщения - 64 байта 
      if (HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &Header, RxData) != HAL_OK){ Error_Handler(); }  
      else{ // напечатаем первые 4 байта входящего сообщения, если все ок. Пример отправки сообщения с Raspberry cansend can0 00000123#DEADBEEF 
      Serial.print("ID ");
      Serial.print(Header.Identifier); // ID сообщения 
      Serial.print(" data: ");
      Serial.print(RxData[0]);
      Serial.print("  ");
      Serial.print(RxData[1]);
      Serial.print("  ");
      Serial.print(RxData[2]);
      Serial.print("  ");
      Serial.print(RxData[3]);
      Serial.println("  ");
    }
    }
  delay(100); 
  
}

  
