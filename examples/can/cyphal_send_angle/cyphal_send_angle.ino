#include <utility>

#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>
#include <uavcan/si/sample/angle/Scalar_1_0.h>

TYPE_ALIAS(AngleScalar, uavcan_si_sample_angle_Scalar_1_0)

static CanFD* canfd;
static FDCAN_HandleTypeDef* hfdcan1;
static std::shared_ptr<CyphalInterface> cyphal_interface;

static bool _is_cyphal_on = false;
static CanardNodeID NODE_ID;

typedef uint32_t millis_t;
typedef uint64_t micros_t;

void error_handler() {
  Serial.println("error");
  while (1) {}
}

UtilityConfig utilities(micros, error_handler);

static constexpr CanardPortID ANGLE_PORT_ID = 1111;
static constexpr micros_t MICROS_S = 1'000'000;

void send_angle(float radian);
void comms_loop(millis_t cur_millis);

#define EACH_N(_value, _counter, N, code_blk) \
  if (((_value) - (_counter)) >= (N)) {       \
    code_blk                                  \
    (_counter) = (_value);                    \
  }

void setup() {
  Serial.begin(115200);

  // Настройка FD CAN
  SystemClock_Config();

  canfd = new CanFD();
  canfd->init();
  canfd->write_default_params();   // 100000 nominal / 8000000 data
  canfd->apply_config();
  hfdcan1 = canfd->get_hfdcan();
  canfd->default_start();

  // Инициализация Cyphal
  NODE_ID = 97;
  size_t queue_len = 200;
  cyphal_interface = CyphalInterface::create_heap<G4CAN, O1Allocator>(
    NODE_ID, hfdcan1, queue_len, utilities
  );

  _is_cyphal_on = true;
}

void loop() {
  uint32_t t = micros();
  millis_t current_millis = (millis_t)(t / 1000);
  comms_loop(current_millis);
}

void send_angle(float radian) {
  if (!_is_cyphal_on) return;

  AngleScalar::Type angle_msg = {};
  static CanardTransferID angle_transfer_id = 0;

  angle_msg.timestamp.microsecond = micros();
  angle_msg.radian = radian;

  cyphal_interface->send_msg<AngleScalar>(
    &angle_msg,
    ANGLE_PORT_ID,
    &angle_transfer_id
  );
}

static millis_t angle_send_time = 0;

void comms_loop(millis_t cur_millis) {
  cyphal_interface->loop();

  EACH_N(cur_millis, angle_send_time, 1000, {
    float some_angle = 1.0f;
    send_angle(some_angle);
  })
}