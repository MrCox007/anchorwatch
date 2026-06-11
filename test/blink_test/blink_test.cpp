#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.println("\n=== ESP8266 Blink Test ===");
  Serial.println("LED should blink every second.");
}

void loop() {
  digitalWrite(LED_BUILTIN, LOW);   // LED ON (active low)
  Serial.println("LED ON");
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);  // LED OFF
  Serial.println("LED OFF");
  delay(500);
}
