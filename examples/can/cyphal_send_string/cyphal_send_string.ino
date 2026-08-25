#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>
#include <cyphal_common_types.hpp>


constexpr CanardNodeID NODE_ID = 97;
constexpr CanardPortID SD_PORT = 176;


CanFD canfd;
std::shared_ptr<ArduinoCyphal<>> cyphal;

static CanardTransferID sd_transfer_id = 0;


void send_string(const char* string){
    CyphalString msg = {};

    size_t len = strlen(string);

    if (len > sizeof(msg.value.elements)){
        len = sizeof(msg.value.elements);
    }

    memcpy(msg.value.elements, string, len);
    msg.value.count = len;

    cyphal->send_msg(&msg, SD_PORT, &sd_transfer_id);
}


void setup(){
    Serial.begin(115200);
    pinMode(LED2, OUTPUT);

    SystemClock_Config();

    canfd.init();
    canfd.write_default_params();
    canfd.apply_config();

    cyphal = make_cyphal<ArduinoCyphal<>>(
        canfd.get_hfdcan(),
        NODE_ID,
        "org.vbcores.string_publisher"
    );

    cyphal->begin();
}


void loop(){
    cyphal->cyphal_loop();

    static uint32_t last_send = 0;

    if (millis() - last_send >= 1000){
        last_send = millis();
        send_string("diagnostic string");
        digitalToggle(LED2);
    }
}