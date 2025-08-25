#include <VBCoreG4_arduino_system.h>

const float ampl = 1.0;
const float freq = 2.0;
float t = 0;
float out = 0;
float curve = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PA4, OUTPUT); // PA4 - DAC pin
  analogWriteResolution(12);

  t = 0;
}

void loop() {
  t += 0.001;
  curve = (ampl*sin(2*M_PI*freq*t) + ampl);
  out = curve * 4096 / 3.3;

  analogWrite(PA4, out);
  // Serial.println(i);

  delay(1);
}
