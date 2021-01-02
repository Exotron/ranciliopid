/********************************************************
  Open ToDos
    HTTP
    Wakeup Timer
    Preinfusion?
******************************************************/
#include <ArduinoOTA.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <ZACwire.h> //NEW TSIC LIB
#include "userConfig.h"
#include "PID_v1.h"

ZACwire<SENSOR_TEMP> TempSensor(306);

double pidInputTemp = 0;
double pidOutput = 0;
double pidSetPoint = SETPOINT;
PID bPID(&pidInputTemp, &pidOutput, &pidSetPoint, AGGKP, (AGGKP / AGGTN), (AGGTV * AGGTN), PONE, DIRECT); //PID initialisation

void ICACHE_RAM_ATTR onTimer1ISR()
{
    static unsigned int isrCounter = 0; // counter for ISR
    timer1_write(6250);                 // set interrupt time to 20ms

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
}

void gpioSetup()
{
    pinMode(RELAY_HEAT, OUTPUT);
    pinMode(RELAY_PUMP, OUTPUT);
    pinMode(RELAY_VALVE, OUTPUT);
    digitalWrite(RELAY_HEAT, LOW);
    digitalWrite(RELAY_PUMP, LOW);
    digitalWrite(RELAY_VALVE, LOW);

    pinMode(SWITCH_MAIN, INPUT);
    pinMode(SWITCH_BREW, INPUT);
    pinMode(SWITCH_WATER, INPUT);
    pinMode(SWITCH_STEAM, INPUT);
}

void setup()
{
    //WLAN und OTA Config
    WiFi.hostname(HOSTNAME);
    WiFi.begin(SSID, PASSWD);
    WiFi.setAutoReconnect(true);
    ArduinoOTA.setHostname(OTAHOST); //  Device name for OTA
    ArduinoOTA.setPassword(OTAPASS); //  Password for OTA
    ArduinoOTA.begin();
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
    timer1_enable(TIM_DIV256, TIM_EDGE, TIM_SINGLE);
    timer1_write(6250); // set interrupt time to 20ms
}

void loop()
{
    static int machineState = 0;
    static bool machineEnabled = false;

    if (machineEnabled == false)
    {
        machineState = STATE_OFF;
    }
    else
    {
        machineState = STATE_ON;
        pidInputTemp = TempSensor.getTemp();

        if (digitalRead(SWITCH_BREW) == HIGH)
        {
            machineState = STATE_BREW;
        }
        else if (digitalRead(SWITCH_WATER) == HIGH)
        {
            machineState = STATE_WATER;
        }
        else if (digitalRead(SWITCH_STEAM) == HIGH)
        {
            machineState = SWITCH_STEAM;
        }
    }

    switch (machineState)
    {
    case STATE_OFF:
        digitalWrite(RELAY_VALVE, LOW);
        digitalWrite(RELAY_PUMP, LOW);
        digitalWrite(RELAY_HEAT, LOW);
        bPID.SetMode(MANUAL);
        pidOutput = 0;
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