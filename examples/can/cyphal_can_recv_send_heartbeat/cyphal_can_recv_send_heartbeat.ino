#include <utility>

// --- Системные библиотеки ---
#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>

// --- Типы сообщений Cyphal ---
#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/node/Health_1_0.h>
#include <uavcan/node/Mode_1_0.h>

/*
===========================================================
 Пример работы с Cyphal (CAN FD)

 Что делает программа:
 1. Инициализирует CAN и Cyphal
 2. Отправляет heartbeat раз в секунду
 3. Принимает heartbeat от других нод
 4. Выводит в Serial:
    - ID удалённой ноды
    - её uptime
===========================================================
*/


// ==========================================================
// 🔧 Вспомогательный класс для размещения объектов в памяти
// (используется из-за ограничений embedded среды)
// ==========================================================
template <class T>
class ReservedObject {
private:
    unsigned char buffer[sizeof(T)];
    T* obj;

public:
    template <class... Args>
    void create(Args&&... args) {
        obj = new (buffer) T(std::forward<Args>(args)...);
    }

    T* operator->() { return obj; }

    ~ReservedObject() {
        obj->~T();
    }
};


// ==========================================================
// 📦 Переименование типов для удобства
// ==========================================================
TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)


// ==========================================================
// ⚙️ Глобальные параметры Cyphal
// ==========================================================
static uint8_t CYPHAL_HEALTH_STATUS = uavcan_node_Health_1_0_NOMINAL;
static uint8_t CYPHAL_MODE          = uavcan_node_Mode_1_0_INITIALIZATION;

static CanardNodeID NODE_ID;                         // ID текущей ноды
static std::shared_ptr<CyphalInterface> cyphal_interface;

static bool _is_cyphal_on = false;
constexpr uint64_t MICROS_S = 1'000'000;
// ==========================================================
// 🚀 Инициализация системы
// ==========================================================
CanFD* canfd;
FDCAN_HandleTypeDef* hfdcan1;




// ==========================================================
// 🧯 Обработчик ошибок
// ==========================================================
void error_handler() {
  Serial.println("Cyphal error!");
  while (1) {}   // зависаем
}

// Утилиты (таймер + обработчик ошибок)
UtilityConfig utilities(micros, error_handler);


// ==========================================================
// ⏳ Переменные времени
// ==========================================================
static uint32_t uptime = 0;

uint32_t t_heartbeat;   // таймер отправки heartbeat
uint32_t t_print;       // таймер вывода в Serial


// ==========================================================
// 📥 Данные последнего принятого heartbeat
// ==========================================================
uint32_t last_remote_uptime = 0;
uint8_t  last_remote_node_id = 0;
bool     heartbeat_received = false;


// ==========================================================
// 📡 Подписчик на heartbeat
// ==========================================================
class HBeatSub : public AbstractSubscription<HBeat> {
public:
    HBeatSub(InterfacePtr interface, CanardPortID port_id)
        : AbstractSubscription<HBeat>(interface, port_id) {}

    // Вызывается при получении сообщения
    void handler(const HBeat::Type& msg, CanardRxTransfer* transfer) override {
        last_remote_uptime  = msg.uptime;
        last_remote_node_id = transfer->metadata.remote_node_id;
        heartbeat_received  = true;
    }
};

// Переменная подписчика
ReservedObject<HBeatSub> hbeat_sub;



void setup() {

  Serial.begin(115200);
  pinMode(LED2, OUTPUT);

  // --------------------------------------------------------
  // 🟡 Инициализация CAN FD
  // --------------------------------------------------------
  SystemClock_Config();

  canfd = new CanFD();
  canfd->init();

  // 1 Mbps nominal / 8 Mbps data
  canfd->write_default_params();
  canfd->apply_config();

  hfdcan1 = canfd->get_hfdcan();
  canfd->default_start();


  // --------------------------------------------------------
  // 🟢 Инициализация Cyphal
  // --------------------------------------------------------
  NODE_ID = 97;           // ID нашей ноды
  size_t queue_len = 200;

  cyphal_interface = std::shared_ptr<CyphalInterface>(
      CyphalInterface::create_heap<G4CAN, O1Allocator>(
          NODE_ID,
          hfdcan1,
          queue_len,
          utilities
      )
  );

  // Подписка на heartbeat (фиксированный порт)
  hbeat_sub.create(
      cyphal_interface,
      uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_
  );

  _is_cyphal_on = true;
  CYPHAL_MODE   = uavcan_node_Mode_1_0_OPERATIONAL;

  // Инициализация таймеров
  t_heartbeat = millis();
  t_print     = millis();
}


// ==========================================================
// 🔁 Основной цикл
// ==========================================================
void loop() {

  // Обработка входящих сообщений Cyphal
  cyphal_interface->loop();


  // --------------------------------------------------------
  // 📤 Отправка heartbeat (1 раз в секунду)
  // --------------------------------------------------------
  if (millis() - t_heartbeat >= 1000) {

    heartbeat();

    digitalToggle(PD2);  // индикация
    t_heartbeat = millis();
  }


  // --------------------------------------------------------
  // 📥 Вывод принятого heartbeat (каждые 100 мс)
  // --------------------------------------------------------
  if (millis() - t_print >= 100) {

    if (heartbeat_received) {
      Serial.print("Node ");
      Serial.print(last_remote_node_id);
      Serial.print(": uptime = ");
      Serial.println(last_remote_uptime);
    }

    t_print = millis();
  }
}


// ==========================================================
// 📡 Функция отправки heartbeat
// ==========================================================
void heartbeat() {

  static CanardTransferID transfer_id = 0;

  // Формируем сообщение
  HBeat::Type msg = {
      .uptime = uptime,
      .health = {CYPHAL_HEALTH_STATUS},
      .mode   = {CYPHAL_MODE}
  };

  uptime += 1;

  // Отправка сообщения
  if (_is_cyphal_on) {
    cyphal_interface->send_msg<HBeat>(
        &msg,
        uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
        &transfer_id,
        MICROS_S * 2   // timeout
    );
  }
}