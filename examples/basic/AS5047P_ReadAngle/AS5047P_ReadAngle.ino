#include <VBCoreG4_arduino_system.h>

// include the library for the AS5047P sensor.
#include <AS5047P.h>


// define the chip select port.
#define AS5047P_CHIP_SELECT_PORT PA_15_ALT1
//            MOSI  MISO  SCLK
SPIClass SPI_3(PC12, PC11, PC10); 

// define the spi bus speed 
#define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

// initialize a new AS5047P sensor object.
AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);

// arduino setup routine
void setup() {
  
  // set the pinmode of the led pin to output.
  pinMode(LED2, OUTPUT);

  // initialize the serial bus for the communication with your pc.
  Serial.begin(115200);

  // initialize the AS5047P sensor and hold if sensor can't be initialized.
  while (!as5047p.initSPI(& SPI_3)) {
    Serial.println(F("Can't connect to the AS5047P sensor! Please check the connection..."));
    delay(3000);
  }

}

// arduino loop routine
void loop() {

  Serial.print("Angle: ");                        // print some text to the serial consol.
  Serial.println(as5047p.readAngleDegree());      // read the angle value from the AS5047P sensor an print it to the serial consol.

  digitalWrite(LED2, !digitalRead(LED2));         // blink led            
  delay(100);                                     

}