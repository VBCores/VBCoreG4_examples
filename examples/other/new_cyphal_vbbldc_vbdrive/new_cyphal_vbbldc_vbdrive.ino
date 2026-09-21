/*
Прошивка управляет двумя обычными моторами SimpleFOC по Cyphal-портам
скорости/угла и одним VBDrive по штатному протоколу VBDrive.

Мотор 1: SimpleFOC, NODE_ID = 1
Мотор 2: SimpleFOC, NODE_ID = 2
Мотор 3: VBDrive,   NODE_ID = 3
*/
#include <VBCoreG4_arduino_system.h>
#include <cyphal.h>
#include <cyphal_common_types.hpp>

/* ============================================================
 *                 ПАРАМЕТРЫ УПРАВЛЕНИЯ
 * ============================================================
 * SIMPLEFOC_K1/K2/K3 - коэффициенты для обычных моторов.
 * По умолчанию коэффициенты не отправляются. Чтобы отправлять их
 * при запуске, поставьте SEND_SIMPLEFOC_K_ON_STARTUP в true.
 */
#define SIMPLEFOC_K1 1.1f
#define SIMPLEFOC_K2 4.0f
#define SIMPLEFOC_K3 0.0f
#define SEND_SIMPLEFOC_K_ON_STARTUP false

#define VBDRIVE_KP 12.0f
#define VBDRIVE_KD 0.5f

#define A    0.7f
#define FREQ 0.5f

/* ============================================================
 *                 ТАЙМЕРЫ
 * ============================================================ */
HardwareTimer *timer_create_func = new HardwareTimer(TIM5);
HardwareTimer *timer_show_data = new HardwareTimer(TIM7);
HardwareTimer *timer_send_command = new HardwareTimer(TIM3);

/* ============================================================
 *              НАСТРОЙКА CYPHAL / CAN
 * ============================================================ */
constexpr CanardNodeID NODE_ID = 42; // Не должен совпадать с ID моторов 1, 2, 3.

constexpr CanardNodeID SIMPLEFOC_1_NODE_ID = 1;
constexpr CanardNodeID SIMPLEFOC_2_NODE_ID = 2;
constexpr CanardNodeID VBDRIVE_NODE_ID = 3;

constexpr CanardPortID SIMPLEFOC_BASE_PORT(const CanardNodeID node_id) {
    return (node_id * 100) + 1000;
}

constexpr CanardPortID SIMPLEFOC_1_ANGULAR_VELOCITY_PORT = SIMPLEFOC_BASE_PORT(SIMPLEFOC_1_NODE_ID);
constexpr CanardPortID SIMPLEFOC_1_ANGLE_PORT = SIMPLEFOC_1_ANGULAR_VELOCITY_PORT + 5;
constexpr CanardPortID SIMPLEFOC_1_K_PORT = SIMPLEFOC_1_ANGULAR_VELOCITY_PORT + 10;
constexpr CanardPortID SIMPLEFOC_1_ANGULAR_VELOCITY_PORT_SUB = SIMPLEFOC_1_ANGULAR_VELOCITY_PORT + 50;

constexpr CanardPortID SIMPLEFOC_2_ANGULAR_VELOCITY_PORT = SIMPLEFOC_BASE_PORT(SIMPLEFOC_2_NODE_ID);
constexpr CanardPortID SIMPLEFOC_2_ANGLE_PORT = SIMPLEFOC_2_ANGULAR_VELOCITY_PORT + 5;
constexpr CanardPortID SIMPLEFOC_2_K_PORT = SIMPLEFOC_2_ANGULAR_VELOCITY_PORT + 10;
constexpr CanardPortID SIMPLEFOC_2_ANGULAR_VELOCITY_PORT_SUB = SIMPLEFOC_2_ANGULAR_VELOCITY_PORT + 50;

constexpr CanardPortID VBDRIVE_FOC_STATE_RX_PORT_ID = 3811;
constexpr CanardPortID VBDRIVE_COMMAND_TX_PORT_ID = 2107 + VBDRIVE_NODE_ID;

CanFD canfd;
std::shared_ptr<ArduinoCyphal<>> cyphal;

static CanardTransferID simplefoc_1_velocity_transfer_id = 0;
static CanardTransferID simplefoc_2_velocity_transfer_id = 0;
static CanardTransferID simplefoc_1_k_transfer_id = 0;
static CanardTransferID simplefoc_2_k_transfer_id = 0;
static CanardTransferID vbdrive_command_transfer_id = 0;

/* ============================================================
 *        ПЕРЕМЕННЫЕ УПРАВЛЕНИЯ МОТОРОМ
 * ============================================================ */
float target_angle = 0.0f;
float target_vel = 0.0f;

float received_angle_motor_1 = 0.0f;
float received_angle_motor_2 = 0.0f;
float received_angle_motor_3 = 0.0f;

float received_velocity_motor_1 = 0.0f;
float received_velocity_motor_2 = 0.0f;
float received_velocity_motor_3 = 0.0f;

/* ============================================================
 *        ФЛАГИ
 * ============================================================ */
volatile bool flag_send_comm = false;
volatile bool flag_show_data = false;

/* ============================================================
 *        ОБЪЯВЛЕНИЯ ФУНКЦИЙ
 * ============================================================ */
void set_flag_send_command();
void set_flag_show_data();
void create_func();
void send_command();
void send_simplefoc_k();

/* ============================================================
 *        ПОДПИСКИ CYPHAL
 * ============================================================ */
void simplefoc_1_velocity_handler(const AngularVelocityUnitScalar& msg, CanardRxTransfer*) {
    received_velocity_motor_1 = msg.radian_per_second;
}

void simplefoc_2_velocity_handler(const AngularVelocityUnitScalar& msg, CanardRxTransfer*) {
    received_velocity_motor_2 = msg.radian_per_second;
}

void simplefoc_1_angle_handler(const AngleUnitScalar& msg, CanardRxTransfer*) {
    received_angle_motor_1 = msg.radian;
}

void simplefoc_2_angle_handler(const AngleUnitScalar& msg, CanardRxTransfer*) {
    received_angle_motor_2 = msg.radian;
}

void vbdrive_foc_state_handler(const FocState& msg, CanardRxTransfer* transfer) {
    if (transfer->metadata.remote_node_id != VBDRIVE_NODE_ID) {
        return;
    }

    received_angle_motor_3 = msg.angle.radian;
    received_velocity_motor_3 = msg.velocity.radian_per_second;
}

void heartbeat_handler(const Heartbeat&, CanardRxTransfer* transfer) {
    const CanardNodeID source_node_id = transfer->metadata.remote_node_id;

    if (source_node_id == SIMPLEFOC_1_NODE_ID) {
        digitalToggle(LED1);
    } else if (source_node_id == SIMPLEFOC_2_NODE_ID || source_node_id == VBDRIVE_NODE_ID) {
        digitalToggle(LED2);
    }
}

/* ============================================================
 *        НАСТРОЙКА CAN И CYPHAL
 * ============================================================ */
void can_config(int ID) {
    SystemClock_Config();

    canfd.init();
    canfd.write_default_params();
    canfd.apply_config();

    cyphal = make_cyphal<ArduinoCyphal<>>(canfd.get_hfdcan(), ID, "org.vbcores.mixed_motor_controller");

    cyphal->subscribe(SIMPLEFOC_1_ANGULAR_VELOCITY_PORT, simplefoc_1_velocity_handler);
    cyphal->subscribe(SIMPLEFOC_1_ANGLE_PORT, simplefoc_1_angle_handler);
    cyphal->subscribe(SIMPLEFOC_2_ANGULAR_VELOCITY_PORT, simplefoc_2_velocity_handler);
    cyphal->subscribe(SIMPLEFOC_2_ANGLE_PORT, simplefoc_2_angle_handler);
    cyphal->subscribe(VBDRIVE_FOC_STATE_RX_PORT_ID, vbdrive_foc_state_handler);
    cyphal->subscribe(uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, heartbeat_handler);

    cyphal->begin();
}

void setup() {
    Serial.begin(115200);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);

    can_config(NODE_ID);

    if (SEND_SIMPLEFOC_K_ON_STARTUP) {
        send_simplefoc_k();
    }

    timer_create_func->pause();
    timer_create_func->setOverflow(1000, HERTZ_FORMAT);
    timer_create_func->attachInterrupt(create_func);
    timer_create_func->refresh();
    timer_create_func->resume();

    timer_show_data->pause();
    timer_show_data->setOverflow(200, HERTZ_FORMAT);
    timer_show_data->attachInterrupt(set_flag_show_data);
    timer_show_data->refresh();
    timer_show_data->resume();

    timer_send_command->pause();
    timer_send_command->setOverflow(1000, HERTZ_FORMAT);
    timer_send_command->attachInterrupt(set_flag_send_command);
    timer_send_command->refresh();
    timer_send_command->resume();
}

void loop() {
    cyphal->cyphal_loop();

    if (flag_show_data) {
        // Три графика в Serial Plotter: углы моторов 1, 2 и 3.
        Serial.print(received_angle_motor_1);
        Serial.print(" ");
        Serial.print(received_angle_motor_2);
        Serial.print(" ");
        Serial.println(received_angle_motor_3);

        flag_show_data = false;
    }

    if (flag_send_comm) {
        send_command();
        flag_send_comm = false;
    }
}

/* ============================================================
 *        ОТПРАВКА КОМАНД МОТОРАМ
 * ============================================================ */
void send_command() {
    AngularVelocityUnitScalar simplefoc_velocity_msg{};
    simplefoc_velocity_msg.radian_per_second = target_vel;

    cyphal->send_msg(
        &simplefoc_velocity_msg,
        SIMPLEFOC_1_ANGULAR_VELOCITY_PORT_SUB,
        &simplefoc_1_velocity_transfer_id
    );

    cyphal->send_msg(
        &simplefoc_velocity_msg,
        SIMPLEFOC_2_ANGULAR_VELOCITY_PORT_SUB,
        &simplefoc_2_velocity_transfer_id
    );

    voltbro_foc_command_1_0 vbdrive_command_msg{};
    vbdrive_command_msg.angle.radian = target_angle;
    vbdrive_command_msg.velocity.radian_per_second = target_vel;
    vbdrive_command_msg._torque.newton_meter = 0;

    vbdrive_command_msg.I_kp.value = 4;
    vbdrive_command_msg.I_ki.value = 1600;
    vbdrive_command_msg.position_feedback_gain.value = VBDRIVE_KP;
    vbdrive_command_msg.velocity_feedback_gain.value = VBDRIVE_KD;

    cyphal->send_msg(
        &vbdrive_command_msg,
        VBDRIVE_COMMAND_TX_PORT_ID,
        &vbdrive_command_transfer_id
    );
}

void fill_k_msg(Float16Array& msg) {
    msg.value.count = 3;
    msg.value.elements[0] = SIMPLEFOC_K1;
    msg.value.elements[1] = SIMPLEFOC_K2;
    msg.value.elements[2] = SIMPLEFOC_K3;
}

void send_simplefoc_k() {
    Float16Array k_msg{};
    fill_k_msg(k_msg);

    cyphal->send_msg(
        &k_msg,
        SIMPLEFOC_1_K_PORT,
        &simplefoc_1_k_transfer_id
    );

    cyphal->send_msg(
        &k_msg,
        SIMPLEFOC_2_K_PORT,
        &simplefoc_2_k_transfer_id
    );
}

/* ============================================================
 *        ГЕНЕРАЦИЯ ЗАДАЮЩЕЙ ТРАЕКТОРИИ
 * ============================================================ */
void create_func() {
    static float amplitude = A;
    static float freq = FREQ;
    static uint32_t t0 = millis();

    uint32_t time_dot = millis() - t0;
    float t = float(time_dot) / 1000.0f;

    // Синус: дает и целевой угол для VBDrive, и целевую скорость для SimpleFOC.
    // target_angle = amplitude * sin(2 * PI * freq * t);
    // target_vel = amplitude * 2 * PI * freq * cos(2 * PI * freq * t);

    // Прямоугольный сигнал. Тут только для VBDrive, т.к в SimpleFoc управление по скорости
    // target_angle = amplitude * ((sin(2 * PI * freq * t) >= 0.0f) ? 1.0f : -1.0f);
    // target_vel = 0.0f;

    // Треугольный сигнал.
    float phase = fmod(freq * t, 1.0f);
    if (phase < 0.25f) {
        target_angle = 4.0f * amplitude * phase;
        target_vel = 4.0f * amplitude * freq;
    } else if (phase < 0.75f) {
        target_angle = 2.0f * amplitude - 4.0f * amplitude * phase;
        target_vel = -4.0f * amplitude * freq;
    } else {
        target_angle = -4.0f * amplitude + 4.0f * amplitude * phase;
        target_vel = 4.0f * amplitude * freq;
    }
}

/* ============================================================
 *        ОБРАБОТЧИКИ ТАЙМЕРОВ
 * ============================================================ */
void set_flag_show_data() {
    flag_show_data = true;
}

void set_flag_send_command() {
    flag_send_comm = true;
}
