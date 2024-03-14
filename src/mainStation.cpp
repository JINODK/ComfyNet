#include <Arduino.h>
//esp32
#include <WiFi.h>
//esp8266
// #include <ESP8266WiFi.h>

WiFiServer server(80);

String payload = "";
unsigned long lastMillis = 0;

void setup() {
    Serial.begin(115200);
    Serial.println("Starting...");
    
    // setup wifi as AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWD, 13, 0, 4);
    Serial.println("AP started: ");
    Serial.println(WiFi.softAPIP());

    // start listening for tcp packets
    server.begin();
    Serial.println("Server started");
}

void loop() {
    WiFiClient client = server.available();
    if (client) {
        Serial.println("Client connected");
        Serial.println("IP: ");
        Serial.println(client.remoteIP());
        lastMillis = millis();
        while (client.connected() && (millis() - lastMillis < 1000)) {
            if (client.available()) {
                // push to string Payload
                char c = client.read();
                payload += c;
            delay(1);
            }
        }
        Serial.println(payload);
        payload = "";
        client.stop();
        Serial.println("Client disconnected");
    }
}