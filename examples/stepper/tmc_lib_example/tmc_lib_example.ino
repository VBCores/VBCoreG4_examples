#include <SPI.h>
#include <TMCStepper.h>
#include <VBCoreG4_arduino_system.h>

//Автор Александра Фомина
//Пример, используя библиотеку TMCStepper, вращает мотором
//Чтобы включить драйвер - отправьте 1 в сериал порт, чтобы отключить - 0
//По нажатию пользовательской кнопки можно менять направление вращения
//Можно менять скорость вращения мотора, задавая время задержки periodStep в сериал порт
//по умолчанию стоит задержка в 800 микросекунд. Увеличивая задержку, вы уменьшаете скорость вращения

#define SD_MODE          PA12 // Mode select
#define SPI_MODE         PA11 // Mode select

#define ENCB_DCEN_CFG4   PB0 // Encoder or CFG4
#define ENCA_DCIN_CFG5   PB1 // Encoder or CFG4
#define ENCN_DCO_CFG6    PB0 // Encoder or CFG4


#define EN_PIN           PC5  // Enable  (LOW = Driver is active)
#define DIR_PIN          PA9 // Direction
#define STEP_PIN         PA8  // Step
#define CS_PIN           PA4 // Chip select
#define SW_MOSI          PA7 // Software Master Out Slave In (MOSI)
#define SW_MISO          PA6 // Software Master In Slave Out (MISO)
#define SW_SCK           PA5 // Software Slave Clock (SCK)


#define R_SENSE 0.075f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

// Select your stepper driver type

//TMC5160Stepper driver(CS_PIN, R_SENSE);
TMC5160Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

int periodStep = 800; // чем меньше период, тем выше скорость

int forward = HIGH;
int prev_btnSt;
int buttonState;

HardwareTimer *timer_ser = new HardwareTimer(TIM3);
HardwareTimer *timer_btn = new HardwareTimer(TIM5);

void setup() {
  Serial.begin(115200);
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(USR_BTN, INPUT_PULLUP);
  digitalWrite(EN_PIN, HIGH);      

  prev_btnSt = digitalRead(USR_BTN);

                                  // Enable one according to your setup
  SPI.begin();                    // SPI drivers
//SERIAL_PORT.begin(115200);      // HW UART drivers
//driver.beginSerial(115200);     // SW UART drivers

  driver.begin();                 //  SPI: Init CS pins and possible SW SPI pins
                                  // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(5);                 // Enables driver in software
  driver.rms_current(1410);        // Set motor RMS current
  driver.microsteps(32);          // Set microsteps to 1/16th

  driver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  driver.pwm_autoscale(true);     // Needed for stealthChop

  driver.TCOOLTHRS(0xFFFFF);
  driver.THIGH(0);

  timer_ser->pause();
  timer_ser->setOverflow(100, HERTZ_FORMAT); 
  timer_ser->attachInterrupt(read_serial);
  timer_ser->refresh();
  timer_ser->resume();

  timer_btn->pause();
  timer_btn->setOverflow(100, HERTZ_FORMAT); 
  timer_btn->attachInterrupt(read_btn);
  timer_btn->refresh();
  timer_btn->resume();

  delay(1000);
}


void loop() {
  for (int i = 0; i < 2000; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(periodStep); // для изменения скорости можно регулировать задержку
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(periodStep);
}
  delay(10); 
}

void read_serial(){
  if (Serial.available() > 0) {
    periodStep = Serial.readString().toInt();
    if (periodStep == 0){
      digitalWrite(EN_PIN, HIGH);
    }
    else if (periodStep == 1){
      digitalWrite(EN_PIN, LOW);
      periodStep = 800;
    }
  }
}

void read_btn(){
  buttonState = digitalRead(USR_BTN);
  if(buttonState == LOW && prev_btnSt == HIGH){
    forward = !forward;
    prev_btnSt = buttonState;
  }
  if(buttonState == HIGH) {prev_btnSt = buttonState;}
  digitalWrite(DIR_PIN, forward); // Установка направления вращения
}