#include <SPI.h>
#include <VBCoreG4_arduino_system.h>

//Автор Александра Фомина

#define SD_MODE          PA12 // Mode select
#define SPI_MODE         PA11 // Mode select

#define ENCB_DCEN_CFG4   PB0 // Encoder or CFG4
#define ENCA_DCIN_CFG5   PB1 // Encoder or CFG4
#define ENCN_DCO_CFG6    PB0 // Encoder or CFG4


#define EN_PIN           PC5  // Enable  (LOW = Driver is active)
#define DIR_PIN          PA9 // Direction
#define STEP_PIN         PA8  // Step
// #define SS           PA4 // Chip select
// #define MOSI          PA7 // Software Master Out Slave In (MOSI)
// #define MISO          PA6 // Software Master In Slave Out (MISO)
// #define SCK           PA5 // Software Slave Clock (SCK)

#define SPI1_NSS_PIN PA4

byte data[5];

void sendData(unsigned long address, unsigned long datagram) {
  //TMC5160 takes 40 bit data: 8 address and 32 data

  delay(100);
  unsigned long i_datagram;

  digitalWrite(SPI1_NSS_PIN, LOW);
  delayMicroseconds(10);

  SPI.transfer(address);

  i_datagram |= SPI.transfer((datagram >> 24) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 16) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram >> 8) & 0xff);
  i_datagram <<= 8;
  i_datagram |= SPI.transfer((datagram) & 0xff);
  digitalWrite(SPI1_NSS_PIN ,HIGH);

  Serial.print("Received: ");
  Serial.println(i_datagram, HEX);
  Serial.print(" from register: ");
  Serial.println(address,HEX);
}

void setup(){
  Serial.begin(115200);
  
  //SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE3));
  SPI.setBitOrder(MSBFIRST); 
  SPI.setDataMode(SPI_MODE3); 
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  pinMode(SPI1_NSS_PIN, OUTPUT);

  SPI.begin();

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(SD_MODE, OUTPUT);
  pinMode(SPI_MODE, OUTPUT);
 
  digitalWrite(SD_MODE, LOW);
  digitalWrite(SPI_MODE, HIGH);
  digitalWrite(EN_PIN, LOW);
  digitalWrite(SPI1_NSS_PIN, HIGH);

  sendData(0x80,0x00000000);      //GCONF

  sendData(0xEC,0x000101D5);      //CHOPCONF: TOFF=5, HSTRT=5, HEND=3, TBL=2, CHM=0 (spreadcycle)
  sendData(0x90,0x00070603);      //IHOLD_IRUN: IHOLD=3, IRUN=10 (max.current), IHOLDDELAY=6
  sendData(0x91,0x0000000A);      //TPOWERDOWN=10

  sendData(0xF0,0x00000000);      // PWMCONF
  //sendData(0xF0,0x000401C8);      //PWM_CONF: AUTO=1, 2/1024 Fclk, Switch amp limit=200, grad=1

  sendData(0xA4,0x000003E8);     //A1=1000
  sendData(0xA5,0x000186A0);     //V1=100000
  sendData(0xA6,0x0000C350);     //AMAX=50000
  sendData(0xA7,0x000186A0);     //VMAX=100000
  sendData(0xAA,0x00000578);     //D1=1400
  sendData(0xAB,0x0000000A);     //VSTOP=10

  sendData(0xA0,0x00000000);     //RAMPMODE=0

  sendData(0xA1,0x00000000);     //XACTUAL=0
  sendData(0xAD,0x00000000);     //XTARGET=0



}

void loop(){
  sendData(0xAD,0x0007D000);     //XTARGET=512000
  delay(2000);
  sendData(0x21,0x00000000);
  sendData(0xAD,0x00000000);     //XTARGET=0
  delay(2000);
  sendData(0x21,0x00000000);
  
}

