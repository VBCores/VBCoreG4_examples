#include <utility>
#include <array>

#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>
#include <uavcan/primitive/array/Real32_1_0.h>

TYPE_ALIAS(ArrayFloat, uavcan_primitive_array_Real32_1_0)

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

static constexpr CanardPortID ARRAY_PORT_ID = 2222;
static constexpr micros_t MICROS_S = 1'000'000;

void comms_loop(millis_t cur_millis);
void send_test_array();

template<typename Container>
void send_numbers(const Container& container);

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
  canfd->write_default_params();   // 1000000 nominal / 8000000 data
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

template<typename Container>
void send_numbers(const Container& container) {
  if (!_is_cyphal_on) return;

  ArrayFloat::Type array_msg = {};
  static CanardTransferID array_transfer_id = 0;

  size_t count = container.size();
  size_t max_count = sizeof(array_msg.value.elements) / sizeof(array_msg.value.elements[0]);

  if (count > max_count) {
    count = max_count;
  }

  array_msg.value.count = count;

  size_t pos = 0;
  for (float elem : container) {
    if (pos >= count) {
      break;
    }
    array_msg.value.elements[pos] = elem;
    pos++;
  }

  cyphal_interface->send_msg<ArrayFloat>(
    &array_msg,
    ARRAY_PORT_ID,
    &array_transfer_id
  );
}

void send_test_array() {
  std::array<float, 3> three_numbers = {1.2f, 3.4f, 5.6f};
  send_numbers(three_numbers);
}

static millis_t array_send_time = 0;

void comms_loop(millis_t cur_millis) {
  cyphal_interface->loop();

  EACH_N(cur_millis, array_send_time, 1000, {
    send_test_array();
  })
}