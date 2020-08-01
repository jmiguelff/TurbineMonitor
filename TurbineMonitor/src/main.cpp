#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include "RTClib.h"
#include "DHT.h"
#include <ClickEncoder.h>
#include <TimerOne.h>

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
 * Liquid Crystal Display
 */
LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

/*
 * Configuration Menu
 */
uint8_t menuSize = 10;
String menu[] = {
    "Menu",
    "Warning Top Val",
    "Warning Bot Val",
    "Alarm Top Val",
    "Alarm Bot Val",
    "Exit",
    "NA",
    "NA",
    "NA",
    ""
};
boolean up = false;
boolean down = false;

/*
 * Encoder Library
 */
#define ENCODER_PINA 8
#define ENCODER_PINB 4
#define ENCODER_BTN A2

ClickEncoder *encoder;
int16_t last, value;
uint8_t btnState = 0;

/*
 *  RTC Library 
 */
RTC_DS3231 rtc;
bool isRTCpresent = false;
char myTimestamp[16] = { 0 };

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
enum State { NORMAL = 0, WARNING, ALARM, MENU};

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
}

/*
 * Interrupt routine to get encoder state
 */
void timerIsr() {
    encoder->service();
}

/*
 * Get rotary encoder
 */
void readRotaryEncoder() {
    value += encoder->getValue();
  
    if (value/2 > last) {
        last = value/2;
        down = true;
        delay(150);
    } 
    else if (value/2 < last) {
        last = value/2;
        up = true;
        delay(150);
    }
}

/*
 * Display menu text
 */
void displayMenu(uint8_t pos) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(menu[pos]);
    lcd.setCursor(0, 1);
    lcd.print(menu[pos + 1]);
}

/*
 * Use menu function
 */
void useMenu() {
    uint8_t currentPos = 0;
    uint8_t btn = 0;
    uint8_t exitNow = 0;

    // Call this update screen
    displayMenu(currentPos);

    while(!exitNow) {
        readRotaryEncoder();
        if (up) Serial.println("UP");
        if (down) Serial.println("DOWN");

        // if (encPos > oldEncPos) {
        //     if (currentPos > 8) {
        //         currentPos = 0;
        //     }
        //     else {
        //         currentPos++;
        //     }
        //     displayMenu(currentPos);
        //     oldEncPos = encPos;
        // }
        // else if (encPos < oldEncPos) {
        //     if (currentPos == 0) {
        //         currentPos = 8;
        //     }
        //     else {
        //         currentPos--;
        //     }
        //     displayMenu(currentPos);
        //     oldEncPos = encPos;
        // }
        // else {
        //     Serial.println("No change");
        // }

        // Update button value

        // Wait one cycle
        // delay(1000);
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

    // Initialize encoder
    Timer1.initialize(1000);
    Timer1.attachInterrupt(timerIsr);

    encoder = new ClickEncoder(ENCODER_PINA, ENCODER_PINB, ENCODER_BTN);
    encoder->setAccelerationEnabled(false);

    last = encoder->getValue();

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

    // Start RTC
    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        isRTCpresent = false;
    }
    else {
        Serial.println("RTC detected");
        isRTCpresent = true;

        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
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

    // Update state
    switch(state) {
    case NORMAL:
        if (btnState == ClickEncoder::Clicked) {
            lastState = state; 
            state = MENU; 
            break;
        }
        
        if (voltage > alarmTopLevel) {
            state = ALARM; 
            break;
        }
        
        if (voltage > warnTopLevel) {
            state = WARNING; 
            break;
        }

        state = NORMAL; 
        break;

    case WARNING:
        if (btnState == ClickEncoder::Clicked) {
            lastState = state; 
            state = MENU; 
            break;
        }

        if (voltage > alarmTopLevel) {
            state = ALARM; 
            break;
        }
        
        if (voltage < warnBotLevel) {
            state = NORMAL; 
            break;
        }
        
        state = WARNING; 
        break;

    case ALARM:
        if (btnState == ClickEncoder::Clicked) {
            lastState = state;
            state = MENU; 
            break;
        }
        
        if (voltage < warnBotLevel) {
            state = NORMAL; 
            break;
        }
        
        if (voltage < alarmBotLevel) {
            state = WARNING; 
            break;
        }

        state = ALARM; 
        break;

    case MENU:
        state = lastState;
        break;
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

        // RTC & LCD Update
        if (isRTCpresent) {
            DateTime now = rtc.now();
            memset(myTimestamp, 0, sizeof(myTimestamp));
            sprintf(myTimestamp, "%d/%d-%d:%d:%d", now.day(), now.month(), now.hour(), now.minute(), now.second());
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print(myTimestamp);
            lcd.setCursor(0, 1);
            lcd.print("Normal");
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
    
    case MENU:
        useMenu();
        break;
    }

    // Button trigger for menu
    btnState = encoder->getButton();

    // Read Temperature and Humidity (TEST)
    //float h = dht.readHumidity();
    //float t = dht.readTemperature();
//
    //if (isnan(h) || isnan(t)) {
    //    Serial.println("Fail to read temperature sensor");
    //}
    //else {
    //    Serial.print("Temp [");
    //    Serial.print(t);
    //    Serial.print(" C] Humidity [");
    //    Serial.print(h);
    //    Serial.println(" %]");
    //}

    // Wait one cycle
    delay(1000);
}