/* TT Motor
GM37-3530-1240-90
DC/12V 44RPM
количество магнитов - 22
*/

#include <VBCoreG4_arduino_system.h>

#define IN1 PA8
#define IN2 PA9
#define SLEEPn PB3
#define USR_BTN PC13
#define VREF PA4

#define Enc_A PB6  
#define Enc_B PC7
#define AIOUT PC1

volatile long count = 0;
volatile long rotations = 0;

float vel = 0;
float current_pos = 0;
float target_pos = 0;

int magnet = 22;
int gear_ratio = 90;
int ipr = magnet * gear_ratio;   // если позиция в 2 раза врёт, поставь *2

float prev_count = 0;

float kx = 30.0;   // коэффициент по позиции
float kv = 2.0;    // торможение по скорости

int duty = 0;
int max_duty = 120;
int min_duty = 50;

float deadzone = 0.05; // радиан

HardwareTimer *timer_move = new HardwareTimer(TIM3);
HardwareTimer *timer_vel = new HardwareTimer(TIM5);

void ISR_A() {
  if (digitalRead(Enc_A) == digitalRead(Enc_B)) {
    count -= 1;
  } else {
    count += 1;
  }

  if (count < -ipr) { 
    rotations += 1;
    count = 0;
  }
  else if (count > ipr) {
    rotations -= 1;
    count = 0;
  }
}

long get_total_count() {
  noInterrupts();
  long total = count + rotations * ipr;
  interrupts();
  return total;
}

float get_position_rad() {
  long total = get_total_count();
  return ((float)total * 2.0 * PI) / ipr;
}

void get_velocity() {
  long total = get_total_count();

  vel = ((total - prev_count) * 2.0 * PI / ipr) / 0.02;
  prev_count = total;
}

void stop_motor() {
  analogWrite(IN1, 255);
  analogWrite(IN2, 255);
}

void drive_motor(int pwm) {
  pwm = constrain(pwm, -max_duty, max_duty);

  if (abs(pwm) < min_duty) {
    stop_motor();
    return;
  }

  if (pwm >= 0) {
    analogWrite(IN1, 255);
    analogWrite(IN2, 255 - pwm);
  } else {
    analogWrite(IN2, 255);
    analogWrite(IN1, 255 - abs(pwm));
  }
}

void move() {
  current_pos = get_position_rad();

  float error = target_pos - current_pos;

  float u = kx * error - kv * vel;

  duty = (int)u;
  duty = constrain(duty, -max_duty, max_duty);

  if (abs(error) < deadzone && abs(vel) < 0.2) {
    stop_motor();
  } else {
    drive_motor(duty);
  }
}

void setup() {
  analogReadResolution(12);

  pinMode(Enc_A, INPUT);
  pinMode(Enc_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(Enc_A), ISR_A, CHANGE);

  pinMode(AIOUT, INPUT);

  pinMode(SLEEPn, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(USR_BTN, INPUT_PULLUP);
  pinMode(VREF, OUTPUT);

  digitalWrite(SLEEPn, HIGH);
  digitalWrite(VREF, HIGH);

  analogWriteFrequency(10000);

  stop_motor();

  Serial.begin(500000);

  target_pos = get_position_rad();

  timer_move->pause();
  timer_move->setOverflow(100, HERTZ_FORMAT); 
  timer_move->attachInterrupt(move);
  timer_move->refresh();
  timer_move->resume();

  timer_vel->pause();
  timer_vel->setOverflow(50, HERTZ_FORMAT);
  timer_vel->attachInterrupt(get_velocity);
  timer_vel->refresh();
  timer_vel->resume();

  delay(1000);

  Serial.println("Position control started");
  Serial.println("Send target position in radians, example: 3.14");
}

void loop() {
  if (Serial.available() > 0) {
    target_pos = Serial.readString().toFloat();

    Serial.print("New target: ");
    Serial.println(target_pos);
  }

  Serial.print("target=");
  Serial.print(target_pos);
  Serial.print(" current=");
  Serial.print(current_pos);
  Serial.print(" vel=");
  Serial.print(vel);
  Serial.print(" duty=");
  Serial.println(duty);

  delay(100);
}