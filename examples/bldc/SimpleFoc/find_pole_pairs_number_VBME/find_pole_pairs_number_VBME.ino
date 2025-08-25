
 /*  
 * For VB BLDC 1.1 vs AS5047p on SPI3 
 */

#include <VBCoreG4_arduino_system.h> 
#include <SimpleFOC.h>


// BLDC motor instance
// its important to put pole pairs number as 1!!!

BLDCMotor motor = BLDCMotor(1);
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10);

// magnetic sensor instance - SPI

SPIClass SPI_3(PC12, PC11, PC10);
MagneticSensorSPI sensor = MagneticSensorSPI(PA15, 14, 0x3FFF);
// magnetic sensor instance - I2C
//MagneticSensorI2C sensor = MagneticSensorI2C(0x36, 12, 0X0C, 4);
// magnetic sensor instance - analog output
// MagneticSensorAnalog sensor = MagneticSensorAnalog(A1, 14, 1020);

void setup() {

  pinMode(PB5, INPUT);
  pinMode(PB3, OUTPUT);
  

  pinMode(PB15, OUTPUT);
  pinMode(PB14, OUTPUT);
  pinMode(PB13, OUTPUT);
  digitalWrite(PB15, HIGH);
  digitalWrite(PB14, HIGH);
  digitalWrite(PB13, HIGH);
  
  digitalWrite(PB3, HIGH);
  // initialise magnetic sensor hardware
  sensor.init(&SPI_3);
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // power supply voltage
  // default 12V
  driver.voltage_power_supply = 16;
  driver.init();
  motor.linkDriver(&driver);

  // initialize motor hardware
  motor.init();

  // monitoring port
  Serial.begin(115200);

  // pole pairs calculation routine
  Serial.println("Pole pairs (PP) estimator");
  Serial.println("-\n");

  float pp_search_voltage = 4; // maximum power_supply_voltage/2
  float pp_search_angle = 6*_PI; // search electrical angle to turn

  // move motor to the electrical angle 0
  motor.controller = MotionControlType::angle_openloop;
  motor.voltage_limit=pp_search_voltage;
  motor.move(0);
  _delay(1000);
  // read the sensor angle
  sensor.update();
  float angle_begin = sensor.getAngle();
  _delay(50);

  // move the motor slowly to the electrical angle pp_search_angle
  float motor_angle = 0;
  while(motor_angle <= pp_search_angle){
    motor_angle += 0.01f;
    sensor.update(); // keep track of the overflow
    motor.move(motor_angle);
    _delay(1);
  }
  _delay(1000);
  // read the sensor value for 180
  sensor.update(); 
  float angle_end = sensor.getAngle();
  _delay(50);
  // turn off the motor
  motor.move(0);
  _delay(1000);

  // calculate the pole pair number
  int pp = round((pp_search_angle)/(angle_end-angle_begin));

  Serial.print(F("Estimated PP : "));
  Serial.println(pp);
  Serial.println(F("PP = Electrical angle / Encoder angle "));
  Serial.print(pp_search_angle*180/_PI);
  Serial.print(F("/"));
  Serial.print((angle_end-angle_begin)*180/_PI);
  Serial.print(F(" = "));
  Serial.println((pp_search_angle)/(angle_end-angle_begin));
  Serial.println();


  // a bit of monitoring the result
  if(pp <= 0 ){
    Serial.println(F("PP number cannot be negative"));
    Serial.println(F(" - Try changing the search_voltage value or motor/sensor configuration."));
    return;
  }else if(pp > 30){
    Serial.println(F("PP number very high, possible error."));
  }else{
    Serial.println(F("If PP is estimated well your motor should turn now!"));
    Serial.println(F(" - If it is not moving try to relaunch the program!"));
    Serial.println(F(" - You can also try to adjust the target voltage using serial terminal!"));
  }


  // set motion control loop to be used
  motor.controller = MotionControlType::torque;
  // set the pole pair number to the motor
  motor.pole_pairs = pp;
  //align sensor and start FOC
  motor.initFOC();
  _delay(1000);

  Serial.println(F("\n Motor ready."));
  Serial.println(F("Set the target voltage using serial terminal:"));
}

// uq voltage
float target_voltage = 2;

void loop() {

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_voltage);

  // communicate with the user
  serialReceiveUserCommand();
}


// utility function enabling serial communication with the user to set the target values
// this function can be implemented in serialEvent function as well
void serialReceiveUserCommand() {

  // a string to hold incoming data
  static String received_chars;

  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the string buffer:
    received_chars += inChar;
    // end of user input
    if (inChar == '\n') {

      // change the motor target
      target_voltage = received_chars.toFloat();
      Serial.print("Target voltage: ");
      Serial.println(target_voltage);

      // reset the command buffer
      received_chars = "";
    }
  }
}
