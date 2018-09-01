#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Bounce2.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "config.h"

WiFiClient wifiClient;
PubSubClient client(wifiClient);
Bounce debouncer = Bounce();
DHT dht(DHT_PIN, DHT_TYPE);

void callback(char *topic, byte *payload, unsigned int length) {
    // Toggling the relay pin
    digitalWrite(RELAY_PIN, HIGH);
    delay(600);
    digitalWrite(RELAY_PIN, LOW);

    // Debugging
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect(HOSTNAME)) {
            Serial.println("connected");
            // Once connected, publish an announcement...
            client.publish(AVAILABILITY_TOPIC, "online", true);
            client.publish(TEMP_AVAILABILITY_TOPIC, "online", true);
            client.publish(HUMIDITY_AVAILABILITY_TOPIC, "online", true);
            // ... and resubscribe (if not the shop)
#ifndef SHOP_GARAGE
            client.subscribe(COMMAND_TOPIC);
#endif
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

void setupOTA() {
    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.setHostname(HOSTNAME);

    ArduinoOTA.onStart([]() { Serial.println("Starting"); });
    ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
        }
    });
    ArduinoOTA.begin();
}

void setup() {
    Serial.begin(115200);
    Serial.println();

    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASSWORD);
    Serial.print("Connecting...");

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    Serial.print("Connected, IP address: ");
    Serial.println(WiFi.localIP());

    client.setServer(MQTT_SERVER, 1883);

    setupOTA();

    // Setting up heartbeat
    pinMode(LED_BUILTIN, OUTPUT);

#ifndef SHOP_GARAGE
    // Setting up relay pin
    digitalWrite(RELAY_PIN, LOW);
    pinMode(RELAY_PIN, OUTPUT);
    client.setCallback(callback);
#endif

    // Setting up reed switch
    pinMode(REED_SWITCH_PIN, INPUT_PULLUP);
    debouncer.attach(REED_SWITCH_PIN);
    debouncer.interval(5);

    // Setup Temp/Humidity Sensor
    dht.begin();
}

void loop() {
    static int lastValue = -1;
    static int loops = 0;

    if (!client.connected()) {
        reconnect();
        // After connecting, send initial state of switch
        if (digitalRead(REED_SWITCH_PIN) == HIGH) {
            client.publish(STATE_TOPIC, "open", true);
        } else {
            client.publish(STATE_TOPIC, "closed", true);
        }
    }

    // Process subscribed MQTT topics
    client.loop();

    // Update the Bounce instance
    debouncer.update();

    // Get the updated value, publish to MQTT if new value
    int value = debouncer.read();
    if (value != lastValue) {
        if (value == HIGH) {
            client.publish(STATE_TOPIC, "open", true);
        } else {
            client.publish(STATE_TOPIC, "closed", true);
        }
        lastValue = value;
    }

    if (loops == NUM_LOOPS_TEMP) {
        client.publish(TEMP_STATE_TOPIC,
                       String(dht.readTemperature(true)).c_str(), true);
        client.publish(HUMIDITY_STATE_TOPIC, String(dht.readHumidity()).c_str(),
                       true);
        loops = 0;
    }

    // Handle OTA
    ArduinoOTA.handle();

    // Heartbeat
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(250);

    loops++;
}
