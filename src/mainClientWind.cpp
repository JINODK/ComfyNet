#define UID "wind1"

#include <Arduino.h>
#include <WiFi.h>

#define ENCODER_PIN 27

unsigned int counter = 0;
float counterAvg = 0;
float windspeed = 0;
unsigned long lastTime = 0;
unsigned long lastMicros = 0;

// setup interrupt on pin 14
void IRAM_ATTR handleInterrupt() {
    if (micros() - lastMicros >= 300) {
        counter++;
        lastMicros = micros();
    }  // else ignore as debounce
}

WiFiClient client;
String payload = "";                   // tcp payload
float lastCount[5] = {0, 0, 0, 0, 0};  // FIFO buffer for temperature

// Wind params
// calib: 1.0 - 7; 1.1 - 9; 1.3 - 12; 1.5 - 15; 1.6 - 17; 1.7 - 20; 1.9 - 24;

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
    pinMode(ENCODER_PIN, INPUT);
    attachInterrupt(ENCODER_PIN, handleInterrupt, RISING);
    pinMode(22, OUTPUT);

    // connect to wifi
    handleWifi();
}

void computeAvgCounter() {
    // validate the counter
    // if any element is 0, push current value to 1st element of buffer and return that value
    for (int i = 0; i < 5; i++) {
        if (lastCount[i] == 0) {
            lastCount[i] = counter;
            counterAvg = counter;
            return;
        }
    }

    // else, calculate average of buffer and push current value to last element of buffer only if it is within 5 degrees of average
    float avg = 0;
    for (int i = 0; i < 5; i++) {
        avg += lastCount[i];
    }
    avg /= 5;
    for (int i = 0; i < 4; i++) {
        lastCount[i] = lastCount[i + 1];  // shift left
    }
    lastCount[4] = counter;

    counterAvg = avg;
    return;
}

void loop() {
    // for every second
    if (millis() - lastTime >= 1000) {
        // read sensor
        computeAvgCounter();

        // print the number of interrupts
        Serial.print(counter);
        Serial.print("\t");
        Serial.println(counterAvg);

        // calculate windspeed
        windspeed = (counterAvg - -10.851063829787236) / 17.872340425531924;

        // edge case: if counterAVG is < 2.5, set windspeed to 0
        if (counterAvg < 2.5) {
            windspeed = 0;
        }

        // print to serial
        Serial.print("Windspeed: ");
        Serial.print(windspeed);
        Serial.println(" m/s");

        // send tcp packet to 192.168.4.1
        if (!client.connected()) {
            if (client.connect("192.168.4.1", 80)) {
                // construct payload
                payload = "?uid=" + String(UID) + "&windspeed=" + String(windspeed);
                client.print(payload + "\r\n");
                // ensure the data is sent
                delay(500);
                client.stop();
            }
        }

        // reset the counter
        counter = 0;
        // reset the timer
        lastTime = millis();
        // blink led
        digitalWrite(22, !digitalRead(22));
    }

    handleWifi();
}