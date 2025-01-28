
#include <SimpleFOC.h>
#include <VBCoreG4_arduino_system.h>


SPIClass SPI_3(PC12, PC11, PC10);
MagneticSensorSPI sensor = MagneticSensorSPI(PA15, 14, 0x3FFF);


BLDCMotor motor = BLDCMotor(15);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10);


float target_velocity = 0;

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }


//float sensitivity = 45.0; // mV/A 
InlineCurrentSense current_sense = InlineCurrentSense(45.0, PC1, PC2, PC3); 
DQCurrent_s dq_currents;

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

  
  sensor.init(&SPI_3);
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = 24;
  driver.pwm_frequency = 25000;    // Частота ШИМ (в Гц)
  driver.init();
  motor.linkDriver(&driver);

// choose FOC modulation
// FOCModulationType::SinePWM; (default)
// FOCModulationType::SpaceVectorPWM;
// FOCModulationType::Trapezoid_120;
// FOCModulationType::Trapezoid_150;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
 // motor.phase_resistance = 0.5;
  motor.controller = MotionControlType::velocity;
  // motor.controller = MotionControlType::torque; // Режим управления по току
  // motor.torque_controller = TorqueControlType::foc_current; // FOC управление

  motor.PID_velocity.P =0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  
  motor.voltage_limit = 24;
  motor.current_limit = 10.0;
  motor.PID_velocity.output_ramp = 1000;

  motor.LPF_velocity.Tf = 0.01f;

  current_sense.linkDriver(&driver);
  current_sense.init();
  motor.linkCurrentSense(&current_sense);
  motor.useMonitoring(Serial);
  motor.init();
  
  motor.initFOC();  
  Serial.print("Zero electric offset: ");
  Serial.println(motor.zero_electric_angle);
  Serial.print("Sensor direction: ");
  Serial.println(motor.sensor_direction == Direction::CW ? "CW" : "CCW");

  command.add('M', doTarget, "target velocity");

  HardwareTimer *timer = new HardwareTimer(TIM3);
  timer->pause(); // останавливаем таймер перед настройкой
  timer->setOverflow(1000, HERTZ_FORMAT); // 1 kHz 
  timer->attachInterrupt(foc_timer); // активируем прерывание
  timer->refresh(); // обнулить таймер 
 

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
  _delay(1000);
  timer->resume(); // запускаем таймер
 
}
String msg1 = "Iq (torque current): ";
String msg2 = " A, Id (magnetizing current): ";

void read_current(){
  dq_currents = current_sense.getFOCCurrents(motor.electrical_angle);
  Serial.print(msg1);
  Serial.print(dq_currents.q, 3);
  Serial.print(msg2);
  Serial.println(dq_currents.d, 3);
}
void foc_timer(){
  motor.loopFOC();
}
void loop() {
  read_current();
  motor.move(target_velocity);
  command.run();
}
