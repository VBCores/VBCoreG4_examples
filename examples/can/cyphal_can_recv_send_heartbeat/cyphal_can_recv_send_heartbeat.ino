#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>
#include <cyphal_common_types.hpp>


constexpr CanardNodeID NODE_ID = 2;

CanFD canfd;
std::shared_ptr<ArduinoCyphal<>> cyphal;

uint32_t t_print;


// ==========================================================
// Данные последнего принятого heartbeat
// ==========================================================
uint32_t last_remote_uptime = 0;
uint8_t  last_remote_node_id = 0;
bool     heartbeat_received = false;


void setup() {

  Serial.begin(19200);
  pinMode(LED2, OUTPUT);

  // --------------------------------------------------------
  // Инициализация CAN FD
  // --------------------------------------------------------
  SystemClock_Config();
  canfd.init();

  // 1 Mbps nominal / 8 Mbps data
  canfd.write_default_params();
  canfd.apply_config();


  // --------------------------------------------------------
  // Инициализация Cyphal
  // --------------------------------------------------------
  cyphal = make_cyphal<ArduinoCyphal<>>(
      canfd.get_hfdcan(),
      NODE_ID,
      "heartbeat_test"
  );

  cyphal->begin();

  // --------------------------------------------------------
  // Подписка на heartbeat
  // --------------------------------------------------------
  cyphal->subscribe<Heartbeat>(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, [](const Heartbeat& msg, CanardRxTransfer* transfer) {
    last_remote_uptime = msg.uptime;
    last_remote_node_id = transfer->metadata.remote_node_id;
    heartbeat_received = true;

    digitalToggle(LED2);
  });

  t_print = millis();
}

void loop() {

  // Обработка Cyphal.
  // В том числе библиотека сама отправляет наш heartbeat.
  cyphal->cyphal_loop();


  // --------------------------------------------------------
  // Вывод последнего принятого heartbeat каждую секунду
  // --------------------------------------------------------
  if (millis() - t_print >= 1000) {

    if (heartbeat_received) {

      Serial.print("Node ");
      Serial.print(last_remote_node_id);

      Serial.print(": uptime = ");
      Serial.println(last_remote_uptime);
    }

    t_print = millis();
  }
}