#ifndef _userConfig_H
#define _userConfig_H

//Wifi
#define HOSTNAME "rancilio"
#define SSID "SSID"
#define PASSWD "PASSWORD"

//OTA
#define OTAHOST "Rancilio" // Name to be shown in ARUDINO IDE Port
#define OTAPASS "otapass"  // Password for OTA updates

//PID - values for offline brewdetection
#define AGGBKP 50 // Kp
#define AGGBTN 0  // Tn
#define AGGBTV 20 // Tv

//PID - offline values
#define PONE 1        // 1 = P_ON_E (default), 0 = P_ON_M (special PID mode, other PID-parameter are needed)
#define SETPOINT 95.0 // Temperatur setpoint
#define SETPOINTSTEAM 130.0
#define AGGKP 69.0  // Kp
#define AGGTN 399.0 // Tn
#define AGGTV 0.0   // Tv
#define WINDOWSIZE 1000

//Coffeemachine States
#define STATE_OFF 0
#define STATE_ON 1
#define STATE_BREW 2
#define STATE_STEAM 3
#define STATE_WATER 4

//PIN BELEGUNG
#define SENSOR_TEMP 2 // TEMP SENSOR PIN                D4 on Board

#define RELAY_VALVE 12 //Output pin for 3-way-valve     D6 on Board
#define RELAY_PUMP 13  //Output pin for pump            D7 on Board
#define RELAY_HEAT 14  //Output pin for heater          D5 on Board
#define RELAY_MAIN 3 // Output pim for main power       RX on Board

//Coffeemachine Switches

#define SWITCH_MAIN 16  //                              D0 on Board
#define SWITCH_BREW 5   //                              D1 on Board
#define SWITCH_WATER 4  //                              D2 on Board
#define SWITCH_STEAM 15 //                              D8 on Board

#endif // _userConfig_H
