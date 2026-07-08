#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>
#include <cyphal_common_types.hpp>

constexpr CanardNodeID NODE_ID = 2;
constexpr CanardPortID ANGULAR_VELOCITY_RX_PORT_ID = 2222;

CanFD canfd;
std::shared_ptr<ArduinoCyphal<>> cyphal;

float velocity = 0.0f;

// Обработчик входящего сообщения
void velocity_handler(const AngularVelocityUnitScalar& msg, CanardRxTransfer*)
{
    velocity = msg.radian_per_second;
    digitalToggle(LED2);

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
        "org.vbcores.angular_velocity_receiver"
    );

    // Подписываемся на сообщения угловой скорости
    cyphal->subscribe(
        ANGULAR_VELOCITY_RX_PORT_ID,
        velocity_handler
    );
    cyphal->begin();
}

void loop()
{
    cyphal->cyphal_loop();
    static uint32_t last_print = 0;

    if (millis() - last_print >= 100)
    {
        last_print = millis();
        Serial.println(velocity);
    }
}