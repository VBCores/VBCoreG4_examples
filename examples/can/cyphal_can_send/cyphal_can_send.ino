#include <utility>

#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>

#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/node/Health_1_0.h>
#include <uavcan/node/Mode_1_0.h>
#include <uavcan/si/unit/angular_velocity/Scalar_1_0.h>
#include <uavcan/si/unit/angle/Scalar_1_0.h>



TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(Angle, uavcan_si_unit_angle_Scalar_1_0)
TYPE_ALIAS(AngularVelocity, uavcan_si_unit_angular_velocity_Scalar_1_0)
static uint8_t CYPHAL_HEALTH_STATUS = uavcan_node_Health_1_0_NOMINAL;
static uint8_t CYPHAL_MODE = uavcan_node_Mode_1_0_INITIALIZATION;
static CanardNodeID NODE_ID;

CanFD* canfd;
FDCAN_HandleTypeDef* hfdcan1;
static bool _is_cyphal_on = false;
static std::shared_ptr<CyphalInterface> cyphal_interface;

typedef uint32_t millis_t;
typedef uint64_t micros_t;
void heartbeat();// объявляем функцию, которая будет отправлять heartbeat, реализуем ее ниже
void comms_loop(millis_t); // объявляем функцию, которая принимает время в милисекундах и по времени выполняет команды, реализуем ее ниже

void error_handler() {Serial.println("error"); while (1) {};}
UtilityConfig utilities(micros, error_handler);

constexpr micros_t MICROS_S = 1'000'000;


static constexpr CanardPortID ANGLE_PORT = 6000; // ID порта, куда будем отправлять угол
static constexpr CanardPortID ANGULAR_VELOCITY_PORT = 7000;// ID порта, куда будем отправлять скорость


void setup() {
  Serial.begin(115200);
  pinMode(LED2, OUTPUT);
  /* Настройка FD CAN
  */
  SystemClock_Config();  // Настройка тактирования
  canfd = new CanFD();  // Создаем управляющий класс
  canfd->init(); // Инициализация CAN
  canfd->write_default_params();  // Записываем дефолтные параметры для FDCAN (500000 nominal / 4000000 data)
  canfd->apply_config();  // Применяем их
  hfdcan1 = canfd->get_hfdcan();  // Сохраняем конфиг
  canfd->default_start();
 
  NODE_ID = 104; // ID ноды
  size_t queue_len = 200;//размер очереди сообщений
  cyphal_interface = CyphalInterface::create_heap<G4CAN, O1Allocator>(NODE_ID, hfdcan1, queue_len, utilities);

  _is_cyphal_on = true;
  CYPHAL_MODE = uavcan_node_Mode_1_0_OPERATIONAL;

}

void loop() {
    uint32_t t = micros(); // берем текущее время в микросекундах
    millis_t current_millis = (millis_t)(t / 1000); // переводим время в милисекунды
    // вызываем функцию под названием comms_loop, которая реализована ниже, в качестве аргумента передаем милисекунды
    comms_loop(current_millis);
}

#define EACH_N(_value, _counter, N, code_blk) \
    if ((_value - _counter) >= (N)) {         \
        code_blk                              \
        _counter = _value;                    \
    }
// функция, которая отправляет heardbeat
void heartbeat() { 
    static uint32_t uptime = 0;
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
// функция, которая отправляет угол
void send_angle() {
    Angle::Type msg = {};
    static CanardTransferID angle_transfer_id = 0;
    msg.radian = 3.14;
    cyphal_interface->send_msg<Angle>(&msg, ANGLE_PORT, &angle_transfer_id);
}
//функция, которая отправляет скорость
void send_angular_velocity() {
    AngularVelocity::Type msg = {};
    static CanardTransferID av_transfer_id = 0;
    msg.radian_per_second = 1.57;
    cyphal_interface->send_msg<AngularVelocity>(&msg, ANGULAR_VELOCITY_PORT, &av_transfer_id);
}

static millis_t odom_report_time = 0;
static millis_t hbeat_time = 0;

// функция, принимает время в милисекундах и смотрит, если прошло 2 милисекунды, вызывает функции отправки угла и скорости по цефалу
//если прошло 1000 милисекунд (1 секунда) - вызывается функция heartbeat
void comms_loop(millis_t cur_millis) {
    cyphal_interface->loop();
    EACH_N(cur_millis, odom_report_time, 2, {
        send_angle();
        send_angular_velocity();
    })
    EACH_N(cur_millis, hbeat_time, 1000, {
        digitalToggle(PD2);
        heartbeat();
    })
}
