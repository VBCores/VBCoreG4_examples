#include <VBCoreG4_arduino_system.h>

#define BUZZER_PIN  PA11
#define VOLTAGE_PIN PA7

#define VOLTAGE_DIVIDER     16.0f
#define ADC_RESOLUTION      4095.0f
#define VREF                3.3f

#define LOW_VOLTAGE_THRESHOLD 12.5f


void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Boat voltage monitor with buzzer");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  analogReadResolution(12);
}


void loop() {
  uint16_t adc = analogRead(VOLTAGE_PIN);

  float voltage_adc =
      (static_cast<float>(adc) / ADC_RESOLUTION) * VREF;

  float voltage_real =
      voltage_adc * VOLTAGE_DIVIDER;

  Serial.print("ADC: ");
  Serial.print(adc);

  Serial.print(" | ADC voltage: ");
  Serial.print(voltage_adc, 3);

  Serial.print(" V | Input voltage: ");
  Serial.print(voltage_real, 2);
  Serial.println(" V");

  if (voltage_real < LOW_VOLTAGE_THRESHOLD) {
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("LOW VOLTAGE! BUZZER ON");
  } else {
    digitalWrite(BUZZER_PIN, LOW);
  }

  delay(500);
}