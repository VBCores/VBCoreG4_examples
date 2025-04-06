/**
 *
 * Пример управления bldc мотором по скорости
 * 1) Настройте конфигурации вашего мотора и датчика (полюса, лимиты по скорости/напряжению и т.п)
 * 2) Загрузите код - обратите внимание, при первой загрузке нужно раскомментировать автоколибровку
 * 3) Через cyphal задайте скорость в рад/сек
 Через cyphal можно также отслеживать текущую скорость мотора (порт 2000) и угол мотора (порт 1000),
 а также задать значения для PID регулятора (порт 3000) - k1, k2, k3 - kp, ki и kd соответственно
 Автоколибровку при каждом включении мотора можно убрать. 
 Для этого:
 1) при первой загрузке программы, после выполнения колибровки, запишите значения zero_electric_angle и sensor.direction
 2) вставьте эти значения вместо указанных по умолчанию
 3) закомментируйте строчку кода, которая отвечает за автоколибровку и 
    раскомментируйте часть кода с настройкой параметров, 
    указав ваши значения zero_electric_angle и sensor.direction
 4) загрузите код снова

 Проверка программы с использованием yakut на Raspberry PI
 y mon - посмотреть что ваше устройство вообще в сети, должны увидеть Node ID 4,
 также обратите внимание на светодиод на контроллере, если он не моргает, значит данные не отправляются
 y pub 2004:uavcan.si.unit.angular_velocity.scalar -- "0.5" - задать скорость 0.5 рад/сек
 y sub 2000:uavcan.si.unit.angular_velocity.scalar - прочитать публикуемую скорость мотора
 y sub 1000:uavcan.si.unit.angle.scalar - прочитать публекуемый угол мотора (от -pi до +pi)
 y pub 3000:uavcan.primitive.array.Real16.1.0 -- "[0.2, 20, 0]" - отправить коэффициенты kp, ki, kd


 */
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

// magnetic sensor instance - SPI
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
static CanardNodeID NODE_ID;

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

static constexpr CanardPortID ANGULAR_VELOCITY_PORT = 2000; // в порт 2000 будет публиковаться текущая скорость bldc мотора
static constexpr CanardPortID ANGLE_PORT = 1000; // в порт 1000 будет публиковаться текущий угол мотора, от -pi до +pi
static constexpr CanardPortID K_PORT = 3000; // c порта 3000 подписчик получает значения коэффициентов pid регулятора
static CanardPortID ANGULAR_VELOCITY_PORT_SUB; // порт, с которого подписчик получает задаваемую скорость, его значение задается в setup 


static millis_t uptime = 0;
millis_t t_heartbeat;
millis_t t_vel;

float target_velocity=0;
float k1 = 0.2f, k2 = 20, k3 = 0;

class VelSub: public AbstractSubscription<AngularVelocity> {
public:
    VelSub(InterfacePtr interface, CanardPortID port_id):
          AbstractSubscription<AngularVelocity>(interface, port_id)
          {};
    void handler(const AngularVelocity::Type& msg, CanardRxTransfer* _) override {
        target_velocity = msg.radian_per_second;
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
        motor.PID_velocity.P =k1;
        motor.PID_velocity.I = k2;
        motor.PID_velocity.D = k3;
        delay(100);
        Serial.println(motor.PID_velocity.P);
        Serial.println(motor.PID_velocity.I);
        Serial.println(motor.PID_velocity.D);
    }
};


ReservedObject<VelSub> vel_sub;
ReservedObject<KSub> k_sub;
float offset_angle;
void setup() {


  Serial.begin(115200);

  pinMode(PB5, INPUT);
  pinMode(PB3, OUTPUT);
  pinMode(PB15, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB13, OUTPUT);

  digitalWrite(PB15, HIGH);
  digitalWrite(PB14, HIGH);
  digitalWrite(PB13, HIGH);
  digitalWrite(PB3, HIGH);

  //init can fd
  SystemClock_Config();  // Настройка тактирования
  canfd = new CanFD();  // Создаем управляющий класс
  canfd->init(); // Инициализация CAN
  canfd->write_default_params();  // Записываем дефолтные параметры для FDCAN (500000 nominal / 4000000 data)
  canfd->apply_config();  // Применяем их
  hfdcan1 = canfd->get_hfdcan();  // Сохраняем конфиг
  canfd->default_start();
 
  NODE_ID = 4; // ID ноды
  ANGULAR_VELOCITY_PORT_SUB = ANGULAR_VELOCITY_PORT + NODE_ID;///2004
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


  sensor.init(&SPI_3);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 24;
  driver.pwm_frequency = 25000;    // Частота ШИМ (в Гц)
  driver.init();
  motor.linkDriver(&driver);

// выберите модуляцию FOC, по умолчанию - SinePWM
// FOCModulationType::SinePWM;
// FOCModulationType::SpaceVectorPWM;
// FOCModulationType::Trapezoid_120;
// FOCModulationType::Trapezoid_150;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::velocity;

  motor.PID_velocity.P =k1;
  motor.PID_velocity.I = k2;
  motor.PID_velocity.D = k3;
  motor.voltage_limit = 24;
  motor.PID_velocity.output_ramp = 1000;

  motor.LPF_velocity.Tf = 0.01f;

  current_sense.linkDriver(&driver);
  current_sense.init();
  motor.linkCurrentSense(&current_sense);
  motor.init();
  
  /* 
  Инициализация FOC c калибровкой!!! 
  Закоментируйте  строки с current_sense.skip_align = ... до  motor.initFOC(); включительно
  Раскомментируйте  следующие 5 строк
  */
  motor.initFOC();  
  Serial.print("Zero electric offset: ");
  Serial.println(motor.zero_electric_angle);
  Serial.print("Sensor direction: ");
  Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");
  
  /* 
  !!! Инициализация FOC БЕЗ калибровки - после первой загрузки программы вставьте свои значения
  для motor.zero_electric_angle и motor.sensor_direction, закомментируйте предыдущие 5 строк с  motor.initFOC() до Serial.println ... включительно
  раскомментируйте 4 строки ниже
  */
  // current_sense.skip_align  = true;
  // motor.zero_electric_angle = 5.90; 
  // motor.sensor_direction = Direction::CW; 
  // motor.initFOC();

  HardwareTimer *timer = new HardwareTimer(TIM3);
  timer->pause(); // останавливаем таймер перед настройкой
  timer->setOverflow(1000, HERTZ_FORMAT); // 1 kHz 
  timer->attachInterrupt(foc_timer); // активируем прерывание
  timer->refresh(); // обнулить таймер 

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using cyphal:"));

  t_vel = t_heartbeat = millis();
  _delay(1000);
  timer->resume(); // запускаем таймер
}

void foc_timer(){
  motor.loopFOC();
}

void loop() {

  cyphal_interface->loop();
  if(millis()-t_heartbeat  >= 1000){
      heartbeat();
      digitalToggle(PD2);
      t_heartbeat = millis();
    }

  if(millis()-t_vel  >= 20){
      send_data();
      t_vel = millis();
    }
  motor.move(target_velocity);

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
float constrainAngle(float x){
    x = fmod(x + _PI, _2PI);
    if (x < 0)
        x += _2PI;
    return x - _PI;
}
void send_data(){
    
    static CanardTransferID av_transfer_id = 0;
    static CanardTransferID angle_transfer_id = 0;

    AngularVelocity::Type msg = {
      .radian_per_second = motor.shaft_velocity
    };

    Angle::Type msg_ang = {
      .radian = constrainAngle(sensor.getAngle()+ _PI)
    };
   

    cyphal_interface->send_msg<Angle>(&msg_ang, ANGLE_PORT, &angle_transfer_id);
    cyphal_interface->send_msg<AngularVelocity>(&msg, ANGULAR_VELOCITY_PORT, &av_transfer_id);
}

