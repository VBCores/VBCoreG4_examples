#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>
#include <cyphal_common_types.hpp>

constexpr CanardNodeID NODE_ID = 2;
constexpr CanardPortID ANGLE_TX_PORT_ID = 1000;

CanFD canfd;
std::shared_ptr<ArduinoCyphal<>> cyphal;

static CanardTransferID angle_transfer_id = 0;

void send_angle(float radian)
{
    AngleSampleScalar msg{};

    msg.timestamp.microsecond = micros();
    msg.radian = radian;

    cyphal->send_msg(
        &msg,
        ANGLE_TX_PORT_ID,
        &angle_transfer_id
    );
}

void setup()
{
    Serial.begin(115200);

    SystemClock_Config();
    canfd.init();
    canfd.write_default_params();
    canfd.apply_config();

    cyphal = make_cyphal<ArduinoCyphal<>>(
        canfd.get_hfdcan(),
        NODE_ID,
        "org.vbcores.angle_publisher"
    );
    cyphal->begin();
}

void loop()
{
    cyphal->cyphal_loop();
    static uint32_t last_send = 0;

    if (millis() - last_send >= 1000) // отправляем сообщение 1 раз в секунду
    {
        last_send = millis();
        send_angle(1.0f); // угол равен 1 радиан
        digitalToggle(LED2); // помигаем светодиодом
    }
}