#include <utility>

#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>

#include <uavcan/node/Heartbeat_1_0.h>
#include <uavcan/node/Health_1_0.h>
#include <uavcan/node/Mode_1_0.h>
#include <uavcan/si/unit/angular_velocity/Scalar_1_0.h>

/*В этом примере используется cyphal для получения скорости по Port ID 2004
Скорость в данном примере - это ШИМ, при помощи которого регулируется напряжение, подаваемое на мотор
Также получаемая скорость эхом публикуется по Port ID  2000
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

TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
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

void heartbeat();

void error_handler() {Serial.println("error"); while (1) {};}
UtilityConfig utilities(micros, error_handler);

constexpr micros_t MICROS_S = 1'000'000;

static constexpr CanardPortID ANGULAR_VELOCITY_PORT = 2000;
static CanardPortID ANGULAR_VELOCITY_PORT_SUB;


static millis_t uptime = 0;
millis_t t_heartbeat;
millis_t t_vel;

float velocity=0;

class VelSub: public AbstractSubscription<AngularVelocity> {
public:
    VelSub(InterfacePtr interface, CanardPortID port_id):
          AbstractSubscription<AngularVelocity>(interface, port_id)
          {};
    void handler(const AngularVelocity::Type& msg, CanardRxTransfer* _) override {
        velocity = msg.radian_per_second;
    }
};
ReservedObject<VelSub> vel_sub;

#define IN1 PA8
#define IN2 PA9
#define SLEEPn PB3
#define VrefPin PA4

void setup() {
  Serial.begin(115200);
  pinMode(LED2, OUTPUT);
  pinMode(PC5, OUTPUT);
  digitalWrite(PC5, HIGH);

  pinMode(PB3, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(VrefPin, OUTPUT);
  
 
  digitalWrite(SLEEPn, HIGH);
  digitalWrite(VrefPin, HIGH);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);



  /* Настройка FD CAN
  */
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
  _is_cyphal_on = true;
  CYPHAL_MODE = uavcan_node_Mode_1_0_OPERATIONAL;

  t_vel = t_heartbeat = millis();
}



void loop() {

    cyphal_interface->loop();

    if(millis()-t_heartbeat  >= 1000){
      heartbeat();
      digitalToggle(PD2);
      t_heartbeat = millis();
    }

    if(millis()-t_vel  >= 1){
      send_angular_velocity(velocity);
      t_vel = millis();
    }
    if (velocity >= 0){
      analogWrite(IN1, 255);
      analogWrite(IN2, 255 - velocity);
    }
    else{
      analogWrite(IN2, 255);
      analogWrite(IN1, 255 - abs(velocity));
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

void send_angular_velocity(float velocity) {
    AngularVelocity::Type msg = {};
    static CanardTransferID av_transfer_id = 0;
    msg.radian_per_second = velocity;
    cyphal_interface->send_msg<AngularVelocity>(&msg, ANGULAR_VELOCITY_PORT, &av_transfer_id);
}

