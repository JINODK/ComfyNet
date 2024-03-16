#define UID "temp0"

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>

#include "Adafruit_BME680.h"
#include "Adafruit_Sensor.h"

#define NTC_PIN 36

struct NTCstruct {
    float voltage;
    float resistance;
    float temperature;
};
struct BMEstruct {
    float temperature;
    float humidity;
    float pressure;
    float gas;
};

struct NTCstruct NTC;
struct BMEstruct BME;

Adafruit_BME680 bme;  // I2C

unsigned long lastTime = 0;

WiFiClient client;
String payload = "";                  // tcp payload
float lastTemp[5] = {0, 0, 0, 0, 0};  // FIFO buffer for temperature

// NTC params
// coupled with 30k resistor
// vcc = 2.22v
// 24.6c = 0.91v

void handleWifi() {
    // if wifi got disconnected or not connected, reconnect
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connecting to WiFi...");
        // try to connect
        WiFi.begin(WIFI_SSID, WIFI_PASSWD);
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
            digitalWrite(22, !digitalRead(22));
        }
        Serial.println("\nConnected to WiFi");
        Serial.println(WiFi.localIP());
        // auto reconnect
        WiFi.persistent(true);
        WiFi.setAutoReconnect(true);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting...");

    // setup hardware
    pinMode(NTC_PIN, INPUT);
    pinMode(2, OUTPUT);

    if (!bme.begin()) {
        Serial.println("Could not find a valid BME680 sensor, check wiring!");
        while (1)
            ;
    }

    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);  // 320*C for 150 ms

    // connect to wifi
    handleWifi();
}

struct NTCstruct readNTC() {
    struct NTCstruct ntc;
    // read and calculate temperature
    ntc.voltage = analogRead(NTC_PIN) / 1024.0 * 0.9934;
    ntc.resistance = 19305 * ntc.voltage / (3.3 - ntc.voltage);
    ntc.temperature = 1.0 / (1.0 / 298.15 + 1.0 / (-7334.906) * (log(ntc.resistance / 10000.0))) - 272.15;

    // validate temperature:
    // if any element is 0, push current temperature to 1st element of buffer and return that temperature
    for (int i = 0; i < 5; i++) {
        if (lastTemp[i] == 0) {
            lastTemp[i] = ntc.temperature;
            return ntc;
        }
    }
    // else, calculate average of buffer and push current temperature to last element of buffer only if it is within 2 degrees of average
    float avg = 0;
    for (int i = 0; i < 5; i++) {
        avg += lastTemp[i];
    }
    avg /= 5;

    if (ntc.temperature < avg + 2 && ntc.temperature > avg - 2) {
        for (int i = 0; i < 4; i++) {
            lastTemp[i] = lastTemp[i + 1];  // shift left
        }
        lastTemp[4] = ntc.temperature;
    }

    ntc.temperature = avg;

    return ntc;
}

struct BMEstruct readBME() {
    struct BMEstruct bmeData;
    if (!bme.performReading()) {
        Serial.println("Failed to poll sensor");
        return bmeData;
    }
    bmeData.temperature = bme.temperature;
    bmeData.humidity = bme.humidity;
    bmeData.pressure = bme.pressure / 100.0;
    bmeData.gas = bme.gas_resistance / 1000.0;
    return bmeData;
}

void loop() {
    // for every half second
    if (millis() - lastTime >= 500) {
        // read sensors
        NTC = readNTC();
        BME = readBME();

        // print to serial
        Serial.println("NTC:");
        Serial.print("Value: ");
        Serial.print(analogRead(NTC_PIN));
        Serial.print(" (0-1023)\t");
        Serial.print("Voltage: ");
        Serial.print(NTC.voltage);
        Serial.print(" V\t");
        Serial.print("Resistance: ");
        Serial.print(NTC.resistance);
        Serial.print(" Ohm\t");
        Serial.print("Temperature: ");
        Serial.print(NTC.temperature);
        Serial.println(" C\t");

        Serial.println("BME:");
        Serial.print("Temperature: ");
        Serial.print(BME.temperature);
        Serial.print(" C\t");
        Serial.print("Humidity: ");
        Serial.print(BME.humidity);
        Serial.print(" %\t");
        Serial.print("Pressure: ");
        Serial.print(BME.pressure);
        Serial.print(" hPa\t");
        Serial.print("Gas: ");
        Serial.print(BME.gas);
        Serial.println(" KOhm\t");
        Serial.println("");

        // send tcp packet to 192.168.4.1
        if (!client.connected()) {
            if (client.connect("192.168.4.1", 80)) {
                // construct payload
                payload = "?uid=" + String(UID) + "&tempNTC=" + String(NTC.temperature) + "&temp=" + String(BME.temperature) + "&hum=" + String(BME.humidity) + "&pres=" + String(BME.pressure) + "&gas=" + String(BME.gas) + "&";
                client.print(payload + "\r\n");
                // ensure the data is sent
                delay(10);
                client.stop();
            }
        }
        // reset the timer
        lastTime = millis();
    }

    handleWifi();
}