#include <utility>

#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>

#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/node/Health_1_0.h>
#include <uavcan/node/Mode_1_0.h>

/* Пример отправки и получения heartbeat по Cyphal CAN

Данный пример:
1. Инициализирует CAN и Cyphal
2. Отправляет heartbeat раз в секунду
3. Принимает heartbeat от других нод
4. Выводит в Serial:
   - ID удалённой ноды
   - uptime, пришедший в heartbeat
*/

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

    T* pointer() {
        return obj;
    }

    T* operator->() {
        return obj;
    }

    ~ReservedObject() {
        obj->~T();
    }
};

// Переименование типов
TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)

static uint8_t CYPHAL_HEALTH_STATUS = uavcan_node_Health_1_0_NOMINAL;
static uint8_t CYPHAL_MODE = uavcan_node_Mode_1_0_INITIALIZATION;
static CanardNodeID NODE_ID;

CanFD* canfd;
FDCAN_HandleTypeDef* hfdcan1;
static bool _is_cyphal_on = false;
static std::shared_ptr<CyphalInterface> cyphal_interface;

typedef uint32_t millis_t;
typedef uint64_t micros_t;

void heartbeat();

void error_handler() {
  Serial.println("error");
  while (1) {}
}

UtilityConfig utilities(micros, error_handler);

constexpr micros_t MICROS_S = 1'000'000;

static millis_t uptime = 0;
millis_t t_heartbeat;
millis_t t_print;

// Переменные для хранения последнего принятого heartbeat
uint32_t last_remote_uptime = 0;
uint8_t last_remote_node_id = 0;
bool heartbeat_received = false;

// Класс подписчика на heartbeat
class HBeatSub : public AbstractSubscription<HBeat> {
public:
    HBeatSub(InterfacePtr interface, CanardPortID port_id) :
        AbstractSubscription<HBeat>(interface, port_id)
    {};

    void handler(const HBeat::Type& msg, CanardRxTransfer* transfer) override {
        last_remote_uptime = msg.uptime;
        last_remote_node_id = transfer->metadata.remote_node_id;
        heartbeat_received = true;
    }
};

ReservedObject<HBeatSub> hbeat_sub;

void setup() {
  Serial.begin(115200);
  pinMode(LED2, OUTPUT);

  /* Настройка FD CAN */
  SystemClock_Config();      // Настройка тактирования
  canfd = new CanFD();       // Создаем управляющий класс
  canfd->init();             // Инициализация CAN
  canfd->write_default_params();  // Записываем дефолтные параметры для FDCAN (1000000 nominal / 8000000 data)
  canfd->apply_config();     // Применяем параметры
  hfdcan1 = canfd->get_hfdcan();
  canfd->default_start();

  NODE_ID = 97;              // ID нашей ноды
  size_t queue_len = 200;    // Размер очереди сообщений

  cyphal_interface = std::shared_ptr<CyphalInterface>(
      CyphalInterface::create_heap<G4CAN, O1Allocator>(
          NODE_ID,
          hfdcan1,
          queue_len,
          utilities
      )
  );

  // Создаем подписчика на heartbeat
  hbeat_sub.create(cyphal_interface, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_);

  _is_cyphal_on = true;
  CYPHAL_MODE = uavcan_node_Mode_1_0_OPERATIONAL;

  t_heartbeat = millis();
  t_print = millis();
}

void loop() {
  cyphal_interface->loop();

  // Раз в секунду отправляем heartbeat
  if (millis() - t_heartbeat >= 1000) {
    heartbeat();
    digitalToggle(PD2);
    t_heartbeat = millis();
  }

  // Раз в 100 мс выводим в Serial последний принятый heartbeat
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

void heartbeat() {
  static CanardTransferID hbeat_transfer_id = 0;

  HBeat::Type heartbeat_msg = {
      .uptime = uptime,
      .health = {CYPHAL_HEALTH_STATUS},
      .mode = {CYPHAL_MODE}
  };

  uptime += 1;

  if (_is_cyphal_on) {
    cyphal_interface->send_msg<HBeat>(
        &heartbeat_msg,
        uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
        &hbeat_transfer_id,
        MICROS_S * 2
    );
  }
}