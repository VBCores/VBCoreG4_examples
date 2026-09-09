#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>
#include <cyphal_common_types.hpp>


#define RED_USR PC6
#define GREEN_USR PB2
#define BLUE_USR PB1
#define EN_USR PB0

#define RED_POWER PC5
#define GREEN_POWER PA7
#define BLUE_POWER PA6
#define EN_POWER PA4

#define BAT1_VOLTAGE PC0
#define BAT2_VOLTAGE PC1

#define BUZZER PA8

static constexpr CanardNodeID NODE_ID = 9;
static constexpr CanardPortID BATTERY1_PORT = 7993;
static constexpr CanardPortID BATTERY2_PORT = 7994;
static constexpr CanardPortID LED_SERVICE_PORT = 172;
static constexpr CanardPortID BEEPER_SERVICE_PORT = 258;

CanFD canfd;
std::shared_ptr<ArduinoCyphal<>> cyphal;

static CanardTransferID battery1_transfer_id = 0;
static CanardTransferID battery2_transfer_id = 0;

bool beeper_service_active = false;
bool beeper_state = false;
uint32_t beeper_toggle_timer = 0;
uint32_t beeper_end_time = 0;
uint32_t beeper_half_period = 0;

enum LedColor {
LED_OFF = 0,
LED_RED = 1,
LED_GREEN = 2,
LED_YELLOW = 3,
LED_BLUE = 4,
LED_MAGENTA = 5,
LED_CYAN = 6,
LED_WHITE = 7
};

void setRgb(uint8_t red_pin, uint8_t green_pin, uint8_t blue_pin, uint8_t color){
digitalWrite(red_pin, color & 0x01 ? HIGH : LOW);
digitalWrite(green_pin, color & 0x02 ? HIGH : LOW);
digitalWrite(blue_pin, color & 0x04 ? HIGH : LOW);
}

void setUserLed(uint8_t color){
setRgb(RED_USR, GREEN_USR, BLUE_USR, color);
}

void setPowerLed(uint8_t color){
setRgb(RED_POWER, GREEN_POWER, BLUE_POWER, color);
}

float readVoltage(uint8_t pin){
uint16_t raw = analogRead(pin);
return raw * 3.3f / 4095.0f * 16.0f;
}

void sendBattery(float voltage, uint8_t battery_number, CanardPortID port, CanardTransferID* transfer_id){
BatteryState msg = {};

msg.voltage.volt = voltage;
msg.current.ampere = 0;
msg.charge.coulomb = 0;
msg.design_capacity.coulomb = 0;
msg.capacity.coulomb = 0;
msg.power_supply_health.value = 0;
msg.power_supply_status.value = 0;
msg.power_supply_technology.value = 2;
msg.is_present.value = voltage >= 4.0f;

msg.location.value.elements[0] = '0' + battery_number;
msg.location.value.count = 1;

msg.serial_number.value.count = 0;

cyphal->send_msg(&msg, port, transfer_id);
}

void led_service_handler(const LEDServiceRequest& request, CanardRxTransfer* transfer){
uint8_t color = 0;

if(request.r.value) color |= LED_RED;
if(request.g.value) color |= LED_GREEN;
if(request.b.value) color |= LED_BLUE;

bool accepted = false;

if(request.interface.value == 0){
setUserLed(color);
accepted = true;
}

if(request.interface.value == 1){
setPowerLed(color);
accepted = true;
}

LEDServiceResponse response{};
response.accepted.value = accepted;
cyphal->send_response(&response, transfer);
}

void beeper_service_handler(const BeeperServiceRequest& request, CanardRxTransfer* transfer){
float duration = request.duration.second;
float frequency = request.frequency.hertz;

if(duration > 0.0f && frequency > 0.0f){
beeper_service_active = true;
beeper_state = true;
beeper_toggle_timer = millis();
beeper_end_time = millis() + (uint32_t)(duration * 1000.0f);
beeper_half_period = (uint32_t)(500.0f / frequency);
if(beeper_half_period == 0) beeper_half_period = 1;
}

BeeperServiceResponse response{};
response.accepted.value = 1;
cyphal->send_response(&response, transfer);
}

void setup(){
SystemClock_Config();

pinMode(RED_USR, OUTPUT);
pinMode(GREEN_USR, OUTPUT);
pinMode(BLUE_USR, OUTPUT);
pinMode(EN_USR, INPUT_PULLUP);

pinMode(RED_POWER, OUTPUT);
pinMode(GREEN_POWER, OUTPUT);
pinMode(BLUE_POWER, OUTPUT);
pinMode(EN_POWER, INPUT_PULLUP);

pinMode(BAT1_VOLTAGE, INPUT_ANALOG);
pinMode(BAT2_VOLTAGE, INPUT_ANALOG);

pinMode(BUZZER, OUTPUT);
pinMode(LED2, OUTPUT);

analogReadResolution(12);

setUserLed(LED_OFF);
setPowerLed(LED_OFF);
digitalWrite(BUZZER, LOW);
digitalWrite(LED2, LOW);

canfd.init();
canfd.write_default_params();
canfd.apply_config();

cyphal = make_cyphal<ArduinoCyphal<>>(canfd.get_hfdcan(), NODE_ID, "org.voltbro.power_board");

cyphal->subscribe<Heartbeat>(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, [](const Heartbeat& msg, CanardRxTransfer* transfer){
digitalToggle(LED2);
});

cyphal->subscribe(LED_SERVICE_PORT, CanardTransferKindRequest, led_service_handler);
cyphal->subscribe(BEEPER_SERVICE_PORT, CanardTransferKindRequest, beeper_service_handler);

cyphal->begin();
}

void loop(){
cyphal->cyphal_loop();

float bat1 = readVoltage(BAT1_VOLTAGE);
float bat2 = readVoltage(BAT2_VOLTAGE);

bool battery_alarm = bat1 < 20.0f && bat2 < 20.0f;

if(battery_alarm){
digitalWrite(BUZZER, HIGH);
}else{
if(beeper_service_active){
if((int32_t)(millis() - beeper_end_time) >= 0){
beeper_service_active = false;
beeper_state = false;
digitalWrite(BUZZER, LOW);
}else if(millis() - beeper_toggle_timer >= beeper_half_period){
beeper_toggle_timer = millis();
beeper_state = !beeper_state;
digitalWrite(BUZZER, beeper_state ? HIGH : LOW);
}
}else{
digitalWrite(BUZZER, LOW);
}
}

static uint32_t battery_timer = 0;
if(millis() - battery_timer >= 50){
battery_timer = millis();
sendBattery(bat1, 1, BATTERY1_PORT, &battery1_transfer_id);
sendBattery(bat2, 2, BATTERY2_PORT, &battery2_transfer_id);
}

delay(10);
}