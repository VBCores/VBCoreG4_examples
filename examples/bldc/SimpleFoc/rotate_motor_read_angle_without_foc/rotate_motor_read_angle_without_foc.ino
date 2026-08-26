
 /* For VB BLDC 1.1 vs AS5047p on SPI3
 */

#include <VBCoreG4_arduino_system.h>
#include <SimpleFOC.h>
#include <AS5047P.h>

// define the chip select port.
#define AS5047P_CHIP_SELECT_PORT PA4
//            MOSI  MISO  SCLK
SPIClass SPI_1(PA7, PA6, PA5);

// define the spi bus speed
#define AS5047P_CUSTOM_SPI_BUS_SPEED 1000000

// initialize a new AS5047P sensor object.
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

// BLDC motor instance
// its important to put pole pairs number as 1!!!
BLDCMotor motor = BLDCMotor(1);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10);

static float motor_angle = 0;
static float angle_begin = 0;
static float pp_search_angle = 2000; // search electrical angle to turn

// uq voltage
float target_voltage = 2;

void setup() {
  // monitoring port
  Serial.begin(115200);

  pinMode(PB5, INPUT);
  pinMode(PB3, OUTPUT);

  pinMode(PA4, OUTPUT);
  digitalWrite(PA4, HIGH); // keep AS5047P CS inactive before SPI init

  pinMode(PB15, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB13, OUTPUT);

  // Keep the power stage quiet while AS5047P starts.
  digitalWrite(PB3, LOW);   // DRV8328B nSLEEP: sleep
  digitalWrite(PB15, LOW);  // INLC off during sensor init
  digitalWrite(PB14, LOW);  // INLB off during sensor init
  digitalWrite(PB13, LOW);  // INLA off during sensor init
  delay(100);

  // initialize the AS5047P sensor before waking the motor driver
  while (!as5047p.initSPI(&SPI_1)) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(3000);
  }
  Serial.println(F("AS5047P sensor ready."));

  // Now wake the driver and restore the intended low-side input state.
  digitalWrite(PB3, HIGH);
  delay(20);
  digitalWrite(PB15, HIGH);
  digitalWrite(PB14, HIGH);
  digitalWrite(PB13, HIGH);

  // power supply voltage
  // default 12V
  driver.voltage_power_supply = 16;
  driver.pwm_frequency = 20000; 
  driver.init();
  motor.linkDriver(&driver);

  // initialize motor hardware
  motor.init();

  // pole pairs calculation routine
  Serial.println("Pole pairs (PP) estimator");
  Serial.println("-\n");

  float pp_search_voltage = 4; // maximum power_supply_voltage/2
 
  // move motor to the electrical angle 0
  motor.controller = MotionControlType::angle_openloop;
  motor.voltage_limit = pp_search_voltage;
  motor.move(0);
  _delay(1000);


  Serial.println(F("\n Motor ready."));
}

void loop() {
  
  if (motor_angle <= pp_search_angle) {
    motor_angle += 0.01f;
    motor.move(motor_angle);
    _delay(1);
  }

  uint16_t raw = as5047p.readAngleRaw();
  float angle_rad = (raw & 0x3FFF) * 2.0f * PI / 16384.0f;

  Serial.println(angle_rad, 6);
  
}
