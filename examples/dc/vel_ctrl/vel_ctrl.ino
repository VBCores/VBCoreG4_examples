/*
  TT Motor
  GM37-3530-1240-90
  DC/12V 44RPM 220909
  Количество магнитов — 22
*/

#include <VBCoreG4_arduino_system.h>

#define Enc_A   PB6
#define Enc_B   PC7

#define IN1     PA8
#define IN2     PA9
#define SLEEPn  PB3
#define VrefPin PA4
#define USR_BTN PC13


//==================== Энкодер ====================

const int magnet = 22;
const int gear_ratio = 90;

// Число отсчётов энкодера на оборот выходного вала
const int cpr = magnet * gear_ratio;  // 1980

volatile long count = 0;
long prev_count = 0;

unsigned long last_ts = 0;


//==================== Скорость ====================

// Максимальная разрешённая целевая скорость
const float MAX_TARGET_VELOCITY = 6.0f;

// Заданная скорость, рад/с
volatile float target_velocity = 0.0f;

// Измеренная скорость, рад/с
volatile float motor_vel = 0.0f;


//==================== PID ====================

// Коэффициенты PID для нормальной скорости
float k1 = 2.5f;
float k2 = 1.5f;
float k3 = 0.0f;

// Пропорциональный коэффициент для малой скорости
float k4 = 1.5f;

float Kp = 0.0f;
float Ki = 0.0f;
float Kd = 0.0f;

float error = 0.0f;
float prev_error = 0.0f;
float d_error = 0.0f;
float pid_sum = 0.0f;

// Оставляем значение из рабочего алгоритма
const float PID_DT = 0.01f;

// На скорости меньше 1 рад/с интегратор отключается
const float LOW_VELOCITY_THRESHOLD = 1.0f;


//==================== Управление ====================

// Максимальное управляющее напряжение
const float MAX_CONTROL_VOLTAGE = 16.0f;

// Минимальное напряжение для преодоления трения
const float MIN_CONTROL_VOLTAGE = 2.4f;

// Текущий PWM: от -4095 до 4095
volatile int u = 0;

// Дополнительная компенсация трения
float pwm_gain = 1.0f;
int pwm_offset = 0;


//==================== Таймеры ====================

// Расчёт скорости — 100 Гц
HardwareTimer* timer_vel = new HardwareTimer(TIM3);

// PID и управление мотором — 1000 Гц
HardwareTimer* timer = new HardwareTimer(TIM5);


//==================== Вспомогательные функции ====================

int signValue(float value) {
  if (value > 0.0f) {
    return 1;
  }

  if (value < 0.0f) {
    return -1;
  }

  return 0;
}


// На STM32 чтение 32-битного long выполняется атомарно
long readCount() {
  return count;
}


// Ограничение целевой скорости диапазоном ±6 рад/с
float limitTargetVelocity(float velocity) {
  if (velocity > MAX_TARGET_VELOCITY) {
    return MAX_TARGET_VELOCITY;
  }

  if (velocity < -MAX_TARGET_VELOCITY) {
    return -MAX_TARGET_VELOCITY;
  }

  return velocity;
}


//==================== Энкодер ====================

void ISR_A() {
  if (digitalRead(Enc_A) == digitalRead(Enc_B)) {
    count += 1;
  }
  else {
    count -= 1;
  }
}


//==================== Расчёт скорости ====================

// Вызывается с частотой 100 Гц
void calc_velocity() {
  unsigned long now = millis();

  float t =
    (now - last_ts) / 1000.0f;

  // Защита от деления на ноль
  if (t <= 0.0f) {
    return;
  }

  last_ts = now;

  long current_count = readCount();
  long delta = current_count - prev_count;

  prev_count = current_count;

  const float TWO_PI_F = 6.28318530718f;

  float rps =
    (float)delta / ((float)cpr * t);

  // Защита от NaN и Inf
  if (!isnan(rps) && isfinite(rps)) {
    motor_vel = rps * TWO_PI_F;
  }
  else {
    motor_vel = 0.0f;
  }
}


//==================== Компенсация трения ====================

void calc_fric_comp() {
  int current_u = u;

  u = (int)(
    signValue(current_u) *
    (pwm_gain * abs(current_u) + pwm_offset)
  );

  // Ограничение 12-битного PWM
  if (u > 4095) {
    u = 4095;
  }
  else if (u < -4095) {
    u = -4095;
  }
}


//==================== PID и управление мотором ====================

// Вызывается с частотой 1000 Гц
void move_motor() {
  float velocity = motor_vel;
  float target = target_velocity;

  // Дополнительная защита цели внутри регулятора
  target = limitTargetVelocity(target);

  if (isnan(velocity) || !isfinite(velocity)) {
    velocity = 0.0f;
    motor_vel = 0.0f;
  }

  // Нулевая команда — остановка и сброс регулятора
  if (fabsf(target) < 0.01f) {
    error = 0.0f;
    prev_error = 0.0f;
    d_error = 0.0f;
    pid_sum = 0.0f;
    u = 0;

    analogWrite(IN1, 4095);
    analogWrite(IN2, 4095);

    return;
  }

  error = target - velocity;

  pid_sum += error * PID_DT;

  if (!isfinite(pid_sum)) {
    pid_sum = 0.0f;
  }

  d_error =
    (error - prev_error) / PID_DT;

  prev_error = error;

  /*
    На малой заданной скорости интегратор отключается.

    При переходе, например, с 6 рад/с на 0.3 рад/с
    старая накопленная ошибка сразу сбрасывается.
  */
  if (fabsf(target) < LOW_VELOCITY_THRESHOLD) {
    Kp = k4;
    Ki = 0.0f;
    Kd = 0.0f;

    pid_sum = 0.0f;
  }
  else {
    Kp = k1;
    Ki = k2;
    Kd = k3;
  }

  float ctrl =
    Kp * error +
    Ki * pid_sum +
    Kd * d_error;

  // Ограничение управляющего напряжения ±16 В
  if (ctrl > MAX_CONTROL_VOLTAGE) {
    ctrl = MAX_CONTROL_VOLTAGE;
  }
  else if (ctrl < -MAX_CONTROL_VOLTAGE) {
    ctrl = -MAX_CONTROL_VOLTAGE;
  }

  // Минимальное напряжение для преодоления трения
  if (target != 0.0f &&
      fabsf(ctrl) < MIN_CONTROL_VOLTAGE) {

    if (ctrl != 0.0f) {
      ctrl =
        signValue(ctrl) * MIN_CONTROL_VOLTAGE;
    }
    else {
      ctrl =
        signValue(target) * MIN_CONTROL_VOLTAGE;
    }
  }

  // Перевод напряжения в 12-битный PWM
  u = (int)(
    ctrl * 4095.0f /
    MAX_CONTROL_VOLTAGE
  );

  calc_fric_comp();

  int current_u = u;

  if (current_u > 0) {
    analogWrite(IN1, 4095);
    analogWrite(IN2, 4095 - current_u);
  }
  else if (current_u < 0) {
    analogWrite(IN2, 4095);
    analogWrite(IN1, 4095 - abs(current_u));
  }
  else {
    analogWrite(IN1, 4095);
    analogWrite(IN2, 4095);
  }
}


//==================== Setup ====================

void setup() {
  pinMode(Enc_A, INPUT);
  pinMode(Enc_B, INPUT);

  attachInterrupt(
    digitalPinToInterrupt(Enc_A),
    ISR_A,
    CHANGE
  );

  pinMode(SLEEPn, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(VrefPin, OUTPUT);
  pinMode(USR_BTN, INPUT_PULLUP);

  // Силовой PWM — 25 кГц, разрешение 12 бит
  analogWriteFrequency(25000);
  analogWriteResolution(12);

  digitalWrite(SLEEPn, HIGH);

  // Настройка ограничения тока
  digitalWrite(VrefPin, 2500);

  // Начальное состояние двигателя
  analogWrite(IN1, 4095);
  analogWrite(IN2, 4095);

  Serial.begin(115200);
  Serial.setTimeout(20);

  last_ts = millis();
  prev_count = readCount();

  //---------- Расчёт скорости: 100 Гц ----------

  timer_vel->pause();
  timer_vel->setOverflow(100, HERTZ_FORMAT);
  timer_vel->attachInterrupt(calc_velocity);
  timer_vel->refresh();
  timer_vel->resume();

  //---------- Управление мотором: 1000 Гц ----------

  timer->pause();
  timer->setOverflow(1000, HERTZ_FORMAT);
  timer->attachInterrupt(move_motor);
  timer->refresh();
  timer->resume();

  Serial.println("Speed control started");
  Serial.println("Target velocity range: -6...6 rad/s");
  Serial.println("Enter target velocity:");
}


//==================== Loop ====================

void loop() {
  if (Serial.available() > 0) {
    float requested_velocity =
      Serial.readString().toFloat();

    float new_target =
      limitTargetVelocity(requested_velocity);

    target_velocity = new_target;

    Serial.print("Requested: ");
    Serial.print(requested_velocity, 3);

    Serial.print(" rad/s | Target: ");
    Serial.print(target_velocity, 3);

    Serial.println(" rad/s");
  }

  // Вывод состояния 10 раз в секунду
  static unsigned long print_timer = 0;

  if (millis() - print_timer >= 100) {
    print_timer = millis();

    float target_copy = target_velocity;
    float velocity_copy = motor_vel;
    int pwm_copy = u;

    Serial.print("Target: ");
    Serial.print(target_copy, 3);

    Serial.print(" rad/s | Velocity: ");
    Serial.print(velocity_copy, 3);

    Serial.print(" rad/s | PWM: ");
    Serial.print(pwm_copy);

    Serial.print(" | Error: ");
    Serial.println(
      target_copy - velocity_copy,
      3
    );
  }
}