#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include "DHT.h"
#include <Adafruit_Sensor.h>

/*
 * DHT Library init
 */
#define DHTTYPE DHT11
#define DHTPIN 7

DHT dht(DHTPIN, DHTTYPE);
float t = 0.0;
float h = 0.0;

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
 * Liquid Crystal Display
 */
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

/*
 * Ethernet Connection Variables
 */
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 41);
IPAddress server(192, 168, 1, 40);

EthernetClient ethClient;
PubSubClient client;
long lastReconnectAttempt = 0;

/*
 *  Json generator
 */
//const size_t capacity = JSON_OBJECT_SIZE(5);
//DynamicJsonDocument doc(capacity);
StaticJsonDocument<60> doc;
char buffer[70] = { 0 };
size_t n = 0;

/*
 * Alarm State Machine Variables
 */
enum State { NORMAL = 0, WARNING, ALARM };

// Initial state
State state = NORMAL;
State lastState = NORMAL;

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
 * Analog Read Variables
 */
float ai0 = 0.0;
float ai1 = 0.0;

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
        // resubscribe
        client.subscribe("warntop");
        client.subscribe("warnbot");
        client.subscribe("alarmtop");
        client.subscribe("alarmbot");

        // Show connected message on LCD
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("MQTT Client");
        lcd.setCursor(0, 1);
        lcd.print("Connected");

    }
    else {
        Serial.print("Fail to connect, ");
        Serial.println(client.state());

        // Show connected message on LCD
        lcd.setCursor(0,0);
        lcd.print("MQTT Client");
        lcd.setCursor(0, 1);
        lcd.print("Fail to connect");
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
    if (strcmp(topic, "warntop") == 0) {
        warnTopLevel = atof((char *) payload);
        Serial.println(warnTopLevel, 2);
    }

    if (strcmp(topic, "warnbot") == 0) {
        warnBotLevel = atof((char *) payload);
        Serial.println(warnBotLevel, 2);
    }

    if (strcmp(topic, "alarmtop") == 0) {
        alarmTopLevel = atof((char *) payload);
        Serial.println(alarmTopLevel, 2);
    }

    if (strcmp(topic, "alarmbot") == 0) {
        alarmBotLevel = atof((char *) payload);
        Serial.println(alarmBotLevel, 2);
    }
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

    // Initialize lcd screen
    lcd.begin();
    lcd.backlight();

    lcd.setCursor(0, 0);
    lcd.print("Booting up");

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
    ai0 = averageMeasure(A0, 60);
    ai1 = averageMeasure(A1, 60);

    // Get temperature and humidity
    h = dht.readHumidity();
    t = dht.readTemperature();

    doc["a"] = ai0;
    doc["b"] = ai1;
    doc["t"] = t;
    doc["h"] = h;

    // Update state
    switch(state) {
    case NORMAL:
        if (ai0 > alarmTopLevel) {
            state = ALARM; 
            break;
        }
        
        if (ai0 > warnTopLevel) {
            state = WARNING; 
            break;
        }

        state = NORMAL; 
        break;

    case WARNING:
        if (ai0 > alarmTopLevel) {
            state = ALARM; 
            break;
        }
        
        if (ai0 < warnBotLevel) {
            state = NORMAL; 
            break;
        }
        
        state = WARNING; 
        break;

    case ALARM:
        if (ai0 < warnBotLevel) {
            state = NORMAL; 
            break;
        }
        
        if (ai0 < alarmBotLevel) {
            state = WARNING; 
            break;
        }

        state = ALARM; 
        break;
    }

    // Update output
    switch(state) {
    case NORMAL:
        digitalWrite(relay1Pin, HIGH);
        digitalWrite(relay2Pin, HIGH);
        rgb_led_write(255, 0, 255); // green light
        doc["s"] = "NORMAL";
        break;

    case WARNING:
        digitalWrite(relay1Pin, LOW);
        digitalWrite(relay2Pin, HIGH);
        rgb_led_write(120, 120, 255); // yellow light
        doc["s"] = "WARNING";
        break;

    case ALARM:
        digitalWrite(relay1Pin, HIGH);
        digitalWrite(relay2Pin, LOW);
        rgb_led_write(100, 255, 255); // red light
        doc["s"] = "ALARM";
        break;
    }

    if (client.connected()) {
        memset(buffer, 0, sizeof(buffer));
        n = serializeJson(doc, buffer);
        client.publish("example", buffer, n);
    }
    else {
        serializeJson(doc, Serial);
        Serial.println();
        serializeJsonPretty(doc, Serial);
    }

    // Wait one cycle
    delay(1000);
}