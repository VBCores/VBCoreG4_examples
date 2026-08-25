#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>
#include <cyphal_common_types.hpp>


constexpr CanardNodeID NODE_ID = 104;

constexpr CanardPortID ANGLE_PORT = 6000;
constexpr CanardPortID ANGULAR_VELOCITY_PORT = 7000;


CanFD canfd;
std::shared_ptr<ArduinoCyphal<>> cyphal;


static CanardTransferID angle_transfer_id = 0;
static CanardTransferID angular_velocity_transfer_id = 0;


void send_angle(){
    AngleUnitScalar msg = {};
    msg.radian = 3.14f;

    cyphal->send_msg(&msg, ANGLE_PORT, &angle_transfer_id);
}


void send_angular_velocity(){
    AngularVelocityUnitScalar msg = {};
    msg.radian_per_second = 1.57f;

    cyphal->send_msg(&msg, ANGULAR_VELOCITY_PORT, &angular_velocity_transfer_id);
}


void setup(){
    Serial.begin(115200);

    SystemClock_Config();

    canfd.init();
    canfd.write_default_params();
    canfd.apply_config();

    cyphal = make_cyphal<ArduinoCyphal<>>(
        canfd.get_hfdcan(),
        NODE_ID,
        "org.vbcores.angle_velocity_publisher"
    );

    cyphal->begin();
}


void loop(){
    cyphal->cyphal_loop();

    static uint32_t last_send = 0;

    if (millis() - last_send >= 2){
        last_send = millis();
        send_angle();
        send_angular_velocity();
    }
}