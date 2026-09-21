#include <SimpleFOC.h>
#include <VBCoreG4_arduino_system.h>
#include <VB_EEPROM.h>

#include <cyphal.h>
#include <cyphal_common_types.hpp>

#define EEPROM_ZERO_ANGLE_ADDR 0x10
#define EEPROM_DIRECTION_ADDR 0x20
#define DIP_1 PB2
#define DIP_2 PB10
#define DIP_3 PB11
#define DIP_4 PB12
#define pinSDA PB_7_ALT1
#define pinSCL PC6

SPIClass SPI_3(PC12, PC11, PC10);
MagneticSensorSPI sensor = MagneticSensorSPI(PA15, 14, 0x3FFF);

BLDCMotor motor = BLDCMotor(15);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10);

InlineCurrentSense current_sense = InlineCurrentSense(45.0, PC1, PC2, PC3);
HardwareTimer* timer = new HardwareTimer(TIM5);

CanFD canfd;
std::shared_ptr<ArduinoCyphal<>> cyphal;

CanardNodeID NODE_ID;
CanardPortID ANGULAR_VELOCITY_PORT;
CanardPortID ANGLE_PORT;
CanardPortID K_PORT;
CanardPortID ANGULAR_VELOCITY_PORT_SUB;

uint32_t t_vel;

float target_velocity = 0;
float k1 = 1.1f, k2 = 4, k3 = 0;
float offset_angle;

void velocity_handler(const AngularVelocityUnitScalar& msg, CanardRxTransfer*) {
  target_velocity = msg.radian_per_second;
}

void k_handler(const Float16Array& msg, CanardRxTransfer*) {
  k1 = msg.value.elements[0];
  k2 = msg.value.elements[1];
  k3 = msg.value.elements[2];
  motor.PID_velocity.P = k1;
  motor.PID_velocity.I = k2;
  motor.PID_velocity.D = k3;
  delay(100);
  Serial.println(motor.PID_velocity.P);
  Serial.println(motor.PID_velocity.I);
  Serial.println(motor.PID_velocity.D);
}

void foc_timer(){
  motor.loopFOC();
}

void set_config(){
  uint8_t mask = digitalRead(DIP_1);
  mask = (mask << 1) ^ digitalRead(DIP_2);
  mask = (mask << 1) ^ digitalRead(DIP_3);
  mask = (mask << 1) ^ digitalRead(DIP_4);
  NODE_ID = (int)mask;
  Serial.print("NODE ID: ");
  Serial.println(NODE_ID);

  ANGULAR_VELOCITY_PORT = (NODE_ID * 100) + 1000;
  ANGLE_PORT = ANGULAR_VELOCITY_PORT + 5;
  K_PORT = ANGULAR_VELOCITY_PORT + 10;
  ANGULAR_VELOCITY_PORT_SUB = ANGULAR_VELOCITY_PORT + 50;

  SystemClock_Config();
  canfd.init();
  canfd.write_default_params();
  canfd.apply_config();

  cyphal = make_cyphal<ArduinoCyphal<>>(canfd.get_hfdcan(), NODE_ID, "org.vbcores.simplefoc_motor");
  cyphal->subscribe(ANGULAR_VELOCITY_PORT_SUB, velocity_handler);
  cyphal->subscribe(K_PORT, k_handler);
  cyphal->begin();
}

void send_data(){
  static CanardTransferID av_transfer_id = 0;
  static CanardTransferID angle_transfer_id = 0;

  AngularVelocityUnitScalar msg{};
  msg.radian_per_second = motor.shaft_velocity;

  AngleUnitScalar msg_ang{};
  msg_ang.radian = sensor.getAngle() - offset_angle;

  cyphal->send_msg(&msg, ANGULAR_VELOCITY_PORT, &av_transfer_id);
  cyphal->send_msg(&msg_ang, ANGLE_PORT, &angle_transfer_id);
  digitalToggle(LED2);
}

void setup() {
  Serial.begin(115200);

  pinMode(LED2, OUTPUT);
  initEEPROM(pinSDA, pinSCL);

  pinMode(USR_BTN, INPUT_PULLUP);
  pinMode(PB5, INPUT);
  pinMode(PB3, OUTPUT);
  pinMode(PB15, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB13, OUTPUT);

  pinMode(DIP_1, INPUT_PULLDOWN);
  pinMode(DIP_2, INPUT_PULLDOWN);
  pinMode(DIP_3, INPUT_PULLDOWN);
  pinMode(DIP_4, INPUT_PULLDOWN);

  digitalWrite(PB15, HIGH);
  digitalWrite(PB14, HIGH);
  digitalWrite(PB13, HIGH);
  digitalWrite(PB3, HIGH);

  set_config();

  sensor.init(&SPI_3);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 24;
  driver.pwm_frequency = 25000;
  driver.init();
  motor.linkDriver(&driver);
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::velocity;

  motor.PID_velocity.P = k1;
  motor.PID_velocity.I = k2;
  motor.PID_velocity.D = k3;
  motor.voltage_limit = 24;

  motor.PID_velocity.output_ramp = 1000;
  motor.LPF_velocity.Tf = 0.125f;

  current_sense.linkDriver(&driver);
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  motor.init();

  if (!isDataInEEPROM(EEPROM_ZERO_ANGLE_ADDR) && !isDataInEEPROM(EEPROM_DIRECTION_ADDR)) {
    Serial.println("EEPROM пуста. Запускаем initFOC");
    motor.initFOC();
    writeFloatToEEPROM(EEPROM_ZERO_ANGLE_ADDR, motor.zero_electric_angle);
    delay(5);
    if (motor.sensor_direction == Direction::CW){writeFloatToEEPROM(EEPROM_DIRECTION_ADDR, 1.0);}
    else {writeFloatToEEPROM(EEPROM_DIRECTION_ADDR, -1.0);}
    delay(5);
  }
  else {
    Serial.print("В EEPROM уже есть значеня.");
    current_sense.skip_align = true;
    motor.zero_electric_angle = readFloatFromEEPROM(EEPROM_ZERO_ANGLE_ADDR);
    if (readFloatFromEEPROM(EEPROM_DIRECTION_ADDR) == 1.0) {motor.sensor_direction = Direction::CW;}
    else if (readFloatFromEEPROM(EEPROM_DIRECTION_ADDR) == -1.0) {motor.sensor_direction = Direction::CCW;}
    else Serial.println("Check EEPROM");
  }

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using cyphal:"));
  offset_angle = sensor.getAngle();
  t_vel = millis();

  timer->pause();
  timer->setOverflow(1000, HERTZ_FORMAT);
  timer->attachInterrupt(foc_timer);
  timer->refresh();
  timer->resume();
}

void loop() {
  cyphal->cyphal_loop();

  if (millis() - t_vel >= 10) {
    send_data();
    t_vel = millis();
  }

  motor.move(target_velocity);
  if (digitalRead(USR_BTN) == 0) clearEEPROM();
  delay(1);
}
