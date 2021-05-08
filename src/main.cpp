/********************************************************
  Open ToDos
    HTTP
    Wakeup Timer
    Preinfusion?
******************************************************/
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ZACwire.h> //NEW TSIC LIB
#include "userConfig.h"
#include "PID_v1.h"
#include "html.h"

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <WebSocketsServer.h> // https://github.com/Links2004/arduinoWebSockets
#include <ESP8266HTTPUpdateServer.h>
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

ZACwire<SENSOR_TEMP> TempSensor(306);

double pidInputTemp = 0;
double pidOutput = 0;
double pidSetPoint = SETPOINT;
PID bPID(&pidInputTemp, &pidOutput, &pidSetPoint, AGGKP, (AGGKP / AGGTN), (AGGTV * AGGTN), PONE, DIRECT); //PID initialisation

MDNSResponder mdns;
WebSocketsServer webSocket = WebSocketsServer(81);
ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

bool machineEnabled = false;

const char *update_path = "/firmware";
const char *update_username = "admin";
const char *update_password = "admin";

void ICACHE_RAM_ATTR onTimer1ISR()
{
    static unsigned int isrCounter = 0; // counter for ISR

    if (pidOutput <= isrCounter)
    {
        digitalWrite(RELAY_HEAT, LOW);
    }
    else
    {
        digitalWrite(RELAY_HEAT, HIGH);
    }

    isrCounter += 20; // += 20 because one tick = 20ms
    //set PID output as relais commands
    if (isrCounter > WINDOWSIZE)
    {
        isrCounter = 0;
    }

    //run PID calculation
    bPID.Compute();

    timer1_write(6250);
}

void gpioSetup()
{
    pinMode(RELAY_HEAT, OUTPUT);
    pinMode(RELAY_PUMP, OUTPUT);
    pinMode(RELAY_VALVE, OUTPUT);
    pinMode(RELAY_MAIN, OUTPUT);
    digitalWrite(RELAY_HEAT, LOW);
    digitalWrite(RELAY_PUMP, LOW);
    digitalWrite(RELAY_VALVE, LOW);

    pinMode(SWITCH_MAIN, INPUT);
    pinMode(SWITCH_BREW, INPUT);
    pinMode(SWITCH_WATER, INPUT);
    pinMode(SWITCH_STEAM, INPUT);
}

void sendUpdate()
{
    DynamicJsonDocument jsonBuffer(1024);

    jsonBuffer["power"] = machineEnabled;

    String res;
    serializeJson(jsonBuffer, res);

    webSocket.broadcastTXT(res);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length)
{
    switch (type)
    {
    case WStype_DISCONNECTED:
        Serial.printf("[%u] Disconnected!\r\n", num);
        break;
    case WStype_CONNECTED:
        Serial.printf("[%u] Connected from url: %s\r\n", num, payload);
        sendUpdate();
        break;
    case WStype_TEXT:
    {
        Serial.printf("[%u] get Text: %s\r\n", num, payload);

        DynamicJsonDocument jsonBuffer(1024);
        DeserializationError error = deserializeJson(jsonBuffer, payload);
        if (error)
        {
            return;
        }

        if (jsonBuffer.containsKey("power"))
        {
            bool power = jsonBuffer["power"];
            machineEnabled = power;
        }
        sendUpdate();
        break;
    }
    case WStype_PING:
        // Serial.printf("[%u] Got Ping!\r\n", num);
        break;
    case WStype_PONG:
        // Serial.printf("[%u] Got Pong!\r\n", num);
        break;
    case WStype_BIN:
        Serial.printf("[%u] get binary length: %u\r\n", num, length);
        break;
    default:
        Serial.printf("Invalid WStype [%d]\r\n", type);
        break;
    }
}

void setup()
{
    Serial.begin(9600); // Start the Serial communication to send messages to the computer
    delay(10);
    Serial.println('\n');
    //WLAN und OTA Config
    WiFi.mode(WIFI_STA);
    WiFi.hostname(HOSTNAME);
    WiFi.begin(SSID, PASSWD);
    WiFi.setAutoReconnect(true);

    Serial.println("Connecting ...");
    int i = 0;
    while (WiFi.status() != WL_CONNECTED)
    { // Wait for the Wi-Fi to connect: scan for Wi-Fi networks, and connect to the strongest of the networks above
        delay(1000);
        Serial.print(++i);
        Serial.print(' ');
    }
    Serial.println('\n');
    Serial.print("Connected to ");
    Serial.println(SSID); // Tell us what network we're connected to
    Serial.print("IP address:\t");
    Serial.println(WiFi.localIP()); // Send the IP address of the ESP8266 to the computer

    if (!MDNS.begin(HOSTNAME))
    { // Start the mDNS responder for esp8266.local
        Serial.println("Error setting up MDNS responder!");
    }
    Serial.println("mDNS responder started");

    // Add service to MDNS-SD
    MDNS.addService("http", "tcp", 80);
    MDNS.addService("oznu-platform", "tcp", 81);
    MDNS.addServiceTxt("oznu-platform", "tcp", "type", "rancilio");
    MDNS.addServiceTxt("oznu-platform", "tcp", "mac", WiFi.macAddress());

    // start web socket
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("Web socket server started on port 81");

    // start http update server
    httpUpdater.setup(&httpServer, update_path, update_username, update_password);

    httpServer.on("/", []() {
        if (!httpServer.authenticate(update_username, update_password))
        {
            return httpServer.requestAuthentication();
        }
        String s = MAIN_page;
        httpServer.send(200, "text/html", s);
    });

    httpServer.begin();

    gpioSetup();
    /********************************************************
     Ini PID
    ******************************************************/
    bPID.SetSampleTime(WINDOWSIZE);
    bPID.SetOutputLimits(0, WINDOWSIZE);
    bPID.SetMode(AUTOMATIC);

    /********************************************************
    Timer1 ISR - Initialisierung
    TIM_DIV1 = 0,   //80MHz (80 ticks/us - 104857.588 us max)
    TIM_DIV16 = 1,  //5MHz (5 ticks/us - 1677721.4 us max)
    TIM_DIV256 = 3  //312.5Khz (1 tick = 3.2us - 26843542.4 us max)
    ******************************************************/
    timer1_isr_init();
    timer1_attachInterrupt(onTimer1ISR);
}

void loop()
{
    MDNS.update();
    httpServer.handleClient();
    webSocket.loop();
    static int machineState = STATE_OFF;
    static int lastMachineState = -1;
    static int lastMillis = 0;

    if (millis() - lastMillis > 500 && digitalRead(SWITCH_MAIN) == HIGH)
    {
        machineEnabled = !machineEnabled;
        lastMillis = millis();
    }

    if (machineEnabled == false)
    {
        machineState = STATE_OFF;
    }
    else
    {
        machineState = STATE_ON;
        pidInputTemp = TempSensor.getTemp();

        if (digitalRead(SWITCH_BREW) == LOW)
        {
            machineState = STATE_BREW;
        }
        else if (digitalRead(SWITCH_WATER) == LOW)
        {
            machineState = STATE_WATER;
        }
        else if (digitalRead(SWITCH_STEAM) == LOW)
        {
            machineState = SWITCH_STEAM;
        }
    }

    if (lastMachineState != machineState && machineState == STATE_OFF)
    {
        lastMachineState = machineState;
        timer1_disable();
        delay(100);
        digitalWrite(RELAY_MAIN, LOW);
    }
    if (lastMachineState != machineState && lastMachineState == STATE_OFF)
    {
        lastMachineState = machineState;
        delay(50);
        digitalWrite(RELAY_MAIN, HIGH);
        delay(50);
        timer1_enable(TIM_DIV256, TIM_EDGE, TIM_SINGLE);
        timer1_write(6250); // set interrupt time to 20ms
    }

    switch (machineState)
    {
    case STATE_OFF:
        bPID.SetMode(MANUAL);
        pidOutput = 0;
        digitalWrite(RELAY_VALVE, LOW);
        digitalWrite(RELAY_PUMP, LOW);
        digitalWrite(RELAY_HEAT, LOW);

        break;
    case STATE_ON:
        digitalWrite(RELAY_VALVE, LOW);
        digitalWrite(RELAY_PUMP, LOW);
        pidSetPoint = SETPOINT;
        bPID.SetMode(AUTOMATIC);

        break;
    case STATE_BREW:
        digitalWrite(RELAY_VALVE, HIGH);
        digitalWrite(RELAY_PUMP, HIGH);
        break;
    case STATE_STEAM:
        digitalWrite(RELAY_VALVE, LOW);
        digitalWrite(RELAY_PUMP, LOW);
        pidSetPoint = SETPOINTSTEAM;
        break;
    case STATE_WATER:
        digitalWrite(RELAY_VALVE, LOW);
        digitalWrite(RELAY_PUMP, HIGH);
        break;
    default:
        digitalWrite(RELAY_VALVE, LOW);
        digitalWrite(RELAY_PUMP, LOW);
        digitalWrite(RELAY_HEAT, LOW);
        bPID.SetMode(MANUAL);
        pidOutput = 0;
        break;
    }
}