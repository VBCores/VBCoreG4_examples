#include <SimpleFOC.h>
#include <VBCoreG4_arduino_system.h>

#include <utility>
#include <cyphal.h>

#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/node/Health_1_0.h>
#include <uavcan/node/Mode_1_0.h>
#include <uavcan/si/unit/angular_velocity/Scalar_1_0.h>
#include <uavcan/si/unit/angle/Scalar_1_0.h>
#include <uavcan/primitive/array/Real16_1_0.h>

#include <Wire.h>
#include <EEPROM.h>

#define EEPROM_I2C_ADDR 0x50  

#define DIP_1 PB2
#define DIP_2 PB10
#define DIP_3 PB11
#define DIP_4 PB12

SPIClass SPI_3(PC12, PC11, PC10);
MagneticSensorSPI sensor = MagneticSensorSPI(PA15, 14, 0x3FFF);

BLDCMotor motor = BLDCMotor(15);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10);

InlineCurrentSense current_sense = InlineCurrentSense(45.0, PC1, PC2, PC3); 


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

TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(AngularVelocity, uavcan_si_unit_angular_velocity_Scalar_1_0)
TYPE_ALIAS(Angle, uavcan_si_unit_angle_Scalar_1_0)
TYPE_ALIAS(ArrayK, uavcan_primitive_array_Real16_1_0)

static uint8_t CYPHAL_HEALTH_STATUS = uavcan_node_Health_1_0_NOMINAL;
static uint8_t CYPHAL_MODE = uavcan_node_Mode_1_0_INITIALIZATION;


CanFD* canfd;
FDCAN_HandleTypeDef* hfdcan1;
static bool _is_cyphal_on = false;
static std::shared_ptr<CyphalInterface> cyphal_interface;

typedef uint32_t millis_t;
typedef uint64_t micros_t;

void heartbeat();
void send_data();

void error_handler() {Serial.println("error"); while (1) {};}
UtilityConfig utilities(micros, error_handler);

constexpr micros_t MICROS_S = 1'000'000;


CanardNodeID NODE_ID;

CanardPortID ANGULAR_VELOCITY_PORT; //2400
CanardPortID ANGLE_PORT;//2405
CanardPortID K_PORT;//2410
CanardPortID ANGULAR_VELOCITY_PORT_SUB; //2450


static millis_t uptime = 0;
millis_t t_heartbeat;
millis_t t_vel;

float target_velocity=0;
float k1 = 1.1f, k2 = 4, k3 = 0;

class VelSub: public AbstractSubscription<AngularVelocity> {
public:
    VelSub(InterfacePtr interface, CanardPortID port_id):
          AbstractSubscription<AngularVelocity>(interface, port_id)
          {};
    void handler(const AngularVelocity::Type& msg, CanardRxTransfer* _) override {
        target_velocity = msg.radian_per_second;
       // Serial.println(target_velocity);
    }
};

class KSub: public AbstractSubscription<ArrayK>{
public:
    KSub(InterfacePtr interface, CanardPortID port_id):
          AbstractSubscription<ArrayK>(interface, port_id)
          {};
    void handler(const ArrayK::Type& msg, CanardRxTransfer* _) override {
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
};



HardwareTimer *timer = new HardwareTimer(TIM5);
ReservedObject<VelSub> vel_sub;
ReservedObject<KSub> k_sub;
float offset_angle;

void foc_timer(){
  motor.loopFOC();
}

void setup() {
  Serial.begin(115200);

  Wire.setSDA(pinSDA); //PB_7_ALT1
  Wire.setSCL(pinSCL); //PC6
  Wire.begin();

  EEPROM.put(0x01, 0xFF); // очистить eeprom
  float zero_electric_angle = 0;
  EEPROM.get(0x01, zero_electric_angle);
  Serial.print("Прочитано значение: ");
  Serial.println(zero_electric_angle);

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
  driver.pwm_frequency = 25000;    // Частота ШИМ (в Гц)
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
  
  /* Инициализация FOC c калибровкой!!! 
  Закоментируйте  строки с current_sense.skip_align = ... до  motor.initFOC(); включительно
  Раскомментируйте  следующие 5 строк*/
  // if (zero_electric_angle){
  //   motor.initFOC();  
  //   Serial.print("Zero electric offset: ");
  //   Serial.println(motor.zero_electric_angle);
  //   Serial.print("Sensor direction: ");
  //   Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
  //   zero_electric_angle = motor.zero_electric_angle;
  //   float sensor_direction;
  //   if(motor.sensor_direction == Direction::CW){
  //     sensor_direction = 1;
  //   }
  //   else{sensor_direction = -1;}
  //   EEPROM.put(0x01, sensor_direction);
  //   EEPROM.put(0x02, zero_electric_angle);

  //   float d, a;
  //   EEPROM.get(0x01, d);
  //   EEPROM.get(0x02, a);
  //   Serial.print(d);
  //   Serial.print(" ");
  //   Serial.println(a);
  // }
  
  
  // current_sense.skip_align  = true;
  // motor.zero_electric_angle = 5.91; //3.23 -- big  5.91 -- small
  // motor.sensor_direction = Direction::CW; 
  // // !!! Инициализация FOC БЕЗ калибровки
  // motor.initFOC();
  
  

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using cyphal:"));
  offset_angle = sensor.getAngle();
  t_vel = t_heartbeat = millis();
  
  timer->pause();
  timer->setOverflow(1000, HERTZ_FORMAT); 
  timer->attachInterrupt(foc_timer);
  timer->refresh();
  timer->resume();

}



void set_config(){
  uint8_t mask = digitalRead(DIP_1);
  mask = (mask<<1)^digitalRead(DIP_2);
  mask = (mask<<1)^digitalRead(DIP_3);
  mask = (mask<<1)^digitalRead(DIP_4);
  NODE_ID = (int)mask;
  Serial.print("NODE ID: ");
  Serial.println(NODE_ID);
  ANGULAR_VELOCITY_PORT = (NODE_ID * 100) + 2000; //2400
  ANGLE_PORT = ANGULAR_VELOCITY_PORT + 5;//2405
  K_PORT = ANGULAR_VELOCITY_PORT + 10;//2410
  ANGULAR_VELOCITY_PORT_SUB = ANGULAR_VELOCITY_PORT + 50; //2450

  //init can fd
  SystemClock_Config();  // Настройка тактирования
  canfd = new CanFD();  // Создаем управляющий класс
  canfd->init(); // Инициализация CAN
  canfd->write_default_params();  // Записываем дефолтные параметры для FDCAN (1000000 nominal / 8000000 data)
  canfd->apply_config();  // Применяем их
  hfdcan1 = canfd->get_hfdcan();  // Сохраняем конфиг
  canfd->default_start();
 
  size_t queue_len = 200;//размер очереди сообщений
  cyphal_interface = std::shared_ptr<CyphalInterface>(CyphalInterface::create_heap<G4CAN, O1Allocator>(
        NODE_ID,
        hfdcan1,
        queue_len,
        utilities
    ));
  vel_sub.create(cyphal_interface, ANGULAR_VELOCITY_PORT_SUB);
  k_sub.create(cyphal_interface, K_PORT);
  _is_cyphal_on = true;
  CYPHAL_MODE = uavcan_node_Mode_1_0_OPERATIONAL;
}

void loop() {

  cyphal_interface->loop();
  
  if(millis() - t_heartbeat >= 1000){
      heartbeat();
      digitalToggle(PD2);
      t_heartbeat = millis();
    }

  if(millis() - t_vel >= 10){
      send_data();
      t_vel = millis();
    }
 // motor.loopFOC();
  motor.move(target_velocity);
  delay(1);
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
// float constrainAngle(float x){
//     x = fmod(x + _PI, _2PI);
//     if (x < 0)
//         x += _2PI;
//     return x - _PI;
// }
void send_data(){
    
    static CanardTransferID av_transfer_id = 0;
    static CanardTransferID angle_transfer_id = 0;

    AngularVelocity::Type msg = {
      .radian_per_second = motor.shaft_velocity
    };

    Angle::Type msg_ang = {
      .radian = sensor.getAngle() - offset_angle//constrainAngle(sensor.getAngle()+ _PI)
    };
   
    cyphal_interface->send_msg<AngularVelocity>(&msg, ANGULAR_VELOCITY_PORT, &av_transfer_id);
    cyphal_interface->send_msg<Angle>(&msg_ang, ANGLE_PORT, &angle_transfer_id);

}

