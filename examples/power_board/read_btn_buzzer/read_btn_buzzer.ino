#include <VBCoreG4_arduino_system.h>

#define RED_USR PC6
#define GREEN_USR PB2
#define BLUE_USR PB1
#define EN_USR PB0

#define RED_POWER PC5
#define GREEN_POWER PA7
#define BLUE_POWER PA6
#define EN_POWER PA4

#define BAT1_VOLTAGE PC0
#define BAT2_VOLTAGE PC1

#define BUZZER PA8

enum LedColor {
LED_OFF = 0,
LED_RED = 1,
LED_GREEN = 2,
LED_YELLOW = 3,
LED_BLUE = 4,
LED_MAGENTA = 5,
LED_CYAN = 6,
LED_WHITE = 7
};

void setRgb(uint8_t red_pin, uint8_t green_pin, uint8_t blue_pin, uint8_t color){
digitalWrite(red_pin, color & 0x01 ? HIGH : LOW);
digitalWrite(green_pin, color & 0x02 ? HIGH : LOW);
digitalWrite(blue_pin, color & 0x04 ? HIGH : LOW);
}

void setUserLed(uint8_t color){
setRgb(RED_USR, GREEN_USR, BLUE_USR, color);
}

void setPowerLed(uint8_t color){
setRgb(RED_POWER, GREEN_POWER, BLUE_POWER, color);
}

float readVoltage(uint8_t pin){
uint16_t raw = analogRead(pin);
return raw * 3.3f / 4095.0f * 16.0f;
}

void setup(){
SystemClock_Config();
Serial.begin(115200);

pinMode(RED_USR, OUTPUT);
pinMode(GREEN_USR, OUTPUT);
pinMode(BLUE_USR, OUTPUT);
pinMode(EN_USR, INPUT_PULLUP);

pinMode(RED_POWER, OUTPUT);
pinMode(GREEN_POWER, OUTPUT);
pinMode(BLUE_POWER, OUTPUT);
pinMode(EN_POWER, INPUT_PULLUP);

pinMode(BAT1_VOLTAGE, INPUT_ANALOG);
pinMode(BAT2_VOLTAGE, INPUT_ANALOG);

pinMode(BUZZER, OUTPUT);

analogReadResolution(12);

setUserLed(LED_OFF);
setPowerLed(LED_OFF);
digitalWrite(BUZZER, LOW);

Serial.println("Format: usr_color power_color");
Serial.println("color: 0=OFF 1=RED 2=GREEN 3=YELLOW 4=BLUE 5=MAGENTA 6=CYAN 7=WHITE");
}

void loop(){
if(Serial.available()){
int usr_color = Serial.parseInt();
int power_color = Serial.parseInt();

if(usr_color >= 0 && usr_color <= 7) setUserLed(usr_color);
if(power_color >= 0 && power_color <= 7) setPowerLed(power_color);

while(Serial.available()) Serial.read();
}

bool usr = digitalRead(EN_USR);
bool power = digitalRead(EN_POWER);

float bat1 = readVoltage(BAT1_VOLTAGE);
float bat2 = readVoltage(BAT2_VOLTAGE);

if(bat1 < 12.0f && bat2 < 12.0f) digitalWrite(BUZZER, HIGH);
else digitalWrite(BUZZER, LOW);

Serial.print(usr);
Serial.print(" ");
Serial.print(power);
Serial.print(" ");
Serial.print(bat1, 2);
Serial.print(" ");
Serial.println(bat2, 2);

delay(10);
}