#include <Arduino.h>

#define NTC_PIN 4

// NTC params
// coupled with 30k resistor
// vcc = 2.22v
// 24.6c = 0.91v

void setup() {
    Serial.begin(115200);
    Serial.println("Starting...");
    pinMode(NTC_PIN, INPUT);
}

void loop() {
    float voltage = analogRead(NTC_PIN) / 1024.0 * 0.9834;
    float resistance = 19305 * voltage / (3.3 - voltage);
    float temperature = 1.0 / (1.0 / 298.15 + 1.0 / (-7334.906)* (log(resistance / 10000.0))) - 273.15;
    Serial.print("Value: ");
    Serial.print(analogRead(NTC_PIN));
    Serial.println(" (0-1023)");
    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.println(" V");
    Serial.print("Resistance: ");
    Serial.print(resistance);
    Serial.println(" Ohm");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" C");
    Serial.println("");
    delay(1000);
}