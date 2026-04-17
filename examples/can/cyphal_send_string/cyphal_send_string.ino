#include <utility>

#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>
#include <uavcan/primitive/String_1_0.h>

TYPE_ALIAS(CyphalString, uavcan_primitive_String_1_0)

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

static constexpr CanardPortID SD_PORT = 176;   // порт для отправки строки
static constexpr micros_t MICROS_S = 1'000'000;

void send_diagnostic(const char* string);
void comms_loop(millis_t cur_millis);

#define EACH_N(_value, _counter, N, code_blk) \
  if (((_value) - (_counter)) >= (N)) {       \
    code_blk                                  \
    (_counter) = (_value);                    \
  }

void setup() {
  Serial.begin(115200);
  pinMode(LED2, OUTPUT);
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

void send_diagnostic(const char* string) {
  if (!_is_cyphal_on) return;

  CyphalString::Type msg = {};
  static CanardTransferID sd_transfer_id = 0;

  size_t len = strlen(string);

  // защита от переполнения массива
  if (len > sizeof(msg.value.elements)) {
    len = sizeof(msg.value.elements);
  }

  memcpy(msg.value.elements, string, len);
  msg.value.count = len;

  cyphal_interface->send_msg<CyphalString>(
    &msg,
    SD_PORT,
    &sd_transfer_id
  );
}

static millis_t diagnostic_time = 0;

void comms_loop(millis_t cur_millis) {
  cyphal_interface->loop();

  EACH_N(cur_millis, diagnostic_time, 1000, {
    send_diagnostic("diagnostic string");
    digitalToggle(LED2);
  })
}