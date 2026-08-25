#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>
#include <cyphal_common_types.hpp>


constexpr CanardNodeID NODE_ID = 97;
constexpr CanardPortID ARRAY_PORT_ID = 2222;


CanFD canfd;
std::shared_ptr<ArduinoCyphal<>> cyphal;

static CanardTransferID array_transfer_id = 0;


void send_test_array(){
    Float32Array msg = {};

    msg.value.count = 3;
    msg.value.elements[0] = 1.2f;
    msg.value.elements[1] = 3.4f;
    msg.value.elements[2] = 5.6f;

    cyphal->send_msg(&msg, ARRAY_PORT_ID, &array_transfer_id);
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
        "org.vbcores.float32_array_publisher"
    );

    cyphal->begin();
}


void loop(){
    cyphal->cyphal_loop();

    static uint32_t last_send = 0;

    if (millis() - last_send >= 1000){
        last_send = millis();
        send_test_array();
    }
}