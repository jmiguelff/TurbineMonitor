#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include "DHT.h"

/*
 * DHT Library init
 */
#define DHTTYPE DHT11
#define DHTPIN 7

DHT dht(DHTPIN, DHTTYPE);

/*
 * Relay Logic Variables
 */
const uint8_t relay1Pin = 2;
const uint8_t relay2Pin = 3;

float warnTopLevel = 2.0;
float warnBotLevel = 1.5;

float alarmTopLevel = 4.0;
float alarmBotLevel = 3.5;

/*
 * LED Logic Consts
 */
const uint8_t redPin = 9;
const uint8_t greenPin = 5;
const uint8_t bluePin = 6;

/*
 * Ethernet Connection Variables
 */
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 2, 20);
IPAddress server(192, 168, 2, 10);

EthernetClient ethClient;
PubSubClient client;

long lastReconnectAttempt = 0;
/*
 * Alarm State Machine Variables
 */
enum { NORMAL = 0, WARNING, ALARM} state;

/*
 * Analog Read Function
 */
float averageMeasure(uint8_t pin, uint8_t points) {
    // Add all measures
    uint16_t sum = 0;
    int sensorValue = 0;
    for (int i = 0; i < points; i++) {
        sensorValue = analogRead(pin);
        sum = sum + sensorValue;
    }

    // Divide by number of measures
    float avg = sum / points;

    // Return value in voltage
    return avg * (5.0 / 1023.0);
}

/*
 * RGB Write
 */
void rgb_led_write(int redValue, int greenValue, int blueValue) {
    analogWrite(redPin, redValue);
    analogWrite(greenPin, greenValue);
    analogWrite(bluePin, blueValue);
}

/*
 * Reconnect to MQTT Broker
 */
boolean reconnect() {
    if (client.connect("monitor01")) {
        Serial.println("Connected to MQTT Broker");
        // Once connected, publish an announcement...
        client.publish("welcomeTopic", "CONNECTED");
        // ... and resubscribe
        client.subscribe("alarmTrigger");
        client.subscribe("warnTrigger");
    }
    else {
        Serial.print("Fail to connect, ");
        Serial.println(client.state());
    }
  
    return client.connected();
}

/*
 * Callback for MQTT Receive Message (for testing)
 */
void msg_rcv_callback(char *topic, byte* payload, unsigned int length) {
    Serial.print("Message arrived [");
    Serial.print(topic);
    Serial.print("] ");
    for (unsigned int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    Serial.println();
}

void setup() {
    // Initialize serial port
    Serial.begin(9600);

    // Initialize digital pins as outputs
    pinMode(relay1Pin, OUTPUT);
    pinMode(relay2Pin, OUTPUT);

    // Initialize LED pins as outputs
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);

    // Intialize DHT library
    dht.begin();

    // Initialize Ethernet device
    Ethernet.begin(mac, ip);
    delay(1500);

    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("Ethernet shield not found - connect shield and reboot");
        while (true) {
            delay(1); // Do nothing forever - until reboot
        }
    }

    if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable is not connected");
    }

    // Initialize MQTT client
    client.setClient(ethClient);
    client.setServer(server, 1883);
    client.setCallback(msg_rcv_callback);

    // Initial state
    state = NORMAL;
}

void loop() {
    // MQTT loop, connect or check messages
    if (!client.connected()) {
        long now = millis();
        if (now - lastReconnectAttempt > 5000) {
            lastReconnectAttempt = now;
            // Attempt to reconnect
            if (reconnect()) {
                lastReconnectAttempt = 0;
            }
        }
    } 
    else {
        // Client connected
        client.loop();
    }

    // ADC read logic
    float voltage = averageMeasure(A0, 60);

    // Print read value
    // Serial.println(voltage);

    // Update state
    switch(state) {
    case NORMAL:
        if (voltage > alarmTopLevel) {state = ALARM; break;}
        if (voltage > warnTopLevel) {state = WARNING; break;}
        state = NORMAL; break;

    case WARNING:
        if (voltage > alarmTopLevel) {state = ALARM; break;}
        if (voltage < warnBotLevel) {state = NORMAL; break;}
        state = WARNING; break;

    case ALARM:
        if (voltage < warnBotLevel) {state = NORMAL; break;}
        if (voltage < alarmBotLevel) {state = WARNING; break;}
        state = ALARM; break;
    }

    // Update output
    switch(state) {
    case NORMAL:
        digitalWrite(relay1Pin, HIGH);
        digitalWrite(relay2Pin, HIGH);
        rgb_led_write(255, 0, 255); // green light
        if (client.connected()) {
            client.publish("state", "NORMAL");
        }
        break;

    case WARNING:
        digitalWrite(relay1Pin, LOW);
        digitalWrite(relay2Pin, HIGH);
        rgb_led_write(0, 0, 255); // yellow light
        if (client.connected()) {
            client.publish("state", "WARNING");
        }
        break;

    case ALARM:
        digitalWrite(relay1Pin, HIGH);
        digitalWrite(relay2Pin, LOW);
        rgb_led_write(0, 255, 255); // yellow light
        if (client.connected()) {
            client.publish("state", "ALARM");
        }
        break;
    }

    // Reaad Temperature and Humidity (TEST)
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t)) {
        Serial.println("Fail to read temperature sensor");
    }
    else {
        Serial.print("Temp [");
        Serial.print(t);
        Serial.print(" C] Humidity [");
        Serial.print(h);
        Serial.println(" %]");
    }

    // Wait one cycle
    delay(1000);
}