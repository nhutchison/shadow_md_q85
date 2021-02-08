// =======================================================================================
//        SHADOW_MD:  Small Handheld Arduino Droid Operating Wand + MarcDuino
// =======================================================================================
//                          Last Revised Date: 01/20/2021
//                             Revised By: TheJuggler
//                Inspired by the PADAWAN / KnightShade SHADOW effort
//                           (Q85 stuff added by Paul Murphy, mostly stolen from Brad/BHD)
// =======================================================================================
//
//         This program is free software: you can redistribute it and/or modify it for
//         your personal use and the personal use of other astromech club members.  
//
//         This program is distributed in the hope that it will be useful 
//         as a courtesy to fellow astromech club members wanting to develop
//         their own droid control system.
//
//         IT IS OFFERED WITHOUT ANY WARRANTY; without even the implied warranty of
//         MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//         You are using this software at your own risk and, as a fellow club member, it is
//         expected you will have the proper experience / background to handle and manage that 
//         risk appropriately.  It is completely up to you to insure the safe operation of
//         your droid and to validate and test all aspects of your droid control system.
//
// =======================================================================================
//   Note: You will need a Arduino Mega ADK rev3 to run this sketch,
//   as a normal Arduino (Uno, Duemilanove etc.) doesn't have enough SRAM and FLASH
//
//   This is written to be a SPECIFIC Sketch - supporting only one type of controller
//      - PS3 Move Navigation + MarcDuino Dome Controller & Optional Body Panel Controller
//
//   This Sketch now supports direct PWM control for Q85 motors running the Roboteq controller
//     with internal mixing or a modified version of BHD's Diamond mixing which now observes
//     the deadzone settings, rather than the hardcoded values there previously.
//     To use the Roboteq mixing, comment out the #define INTERNAL_MIXING below.
//
//     To reverse a motor direction change either #define invertLeft/invertRight to 1
//
//   PS3 Bluetooth library - developed by Kristian Lauszus (kristianl@tkjelectronics.com)
//   For more information visit my blog: http://blog.tkjelectronics.dk/ or
//
//   Sabertooth (Foot Drive):
//         Set Sabertooth 2x32 or 2x25 Dip Switches: 1 and 2 Down, All Others Up
//
//   SyRen 10 Dome Drive:
//         For SyRen packetized Serial Set Switches: 1, 2 and 4 Down, All Others Up
//         NOTE:  Support for SyRen Simple Serial has been removed, due to problems.
//         Please contact DimensionEngineering to get an RMA to flash your firmware
//         Some place a 10K ohm resistor between S1 & GND on the SyRen 10 itself
//
// =======================================================================================
//
// ---------------------------------------------------------------------------------------
//                        General User Settings
// ---------------------------------------------------------------------------------------

#define FOOT_CONTROLLER 1 //0 for Sabertooth Serial or 1 for individual R/C output (for Q85 motors with 1 controller for each foot, or Sabertooth Mode 2 Independant Mixing)
#define INTERNAL_MIXING

String PS3ControllerFootMac = "XX:XX:XX:XX:XX:XX";  //Set this to your FOOT PS3 controller MAC address
//String PS3ControllerFootMac = "00:07:04:0B:3E:7B";  //Set this to your FOOT PS3 controller MAC address
String PS3ControllerDomeMAC = "XX:XX:XX:XX:XX:XX";  //Set to a secondary DOME PS3 controller MAC address (Optional)

String PS3ControllerBackupFootMac = "XX";  //Set to the MAC Address of your BACKUP FOOT controller (Optional)
String PS3ControllerBackupDomeMAC = "XX";  //Set to the MAC Address of your BACKUP DOME controller (Optional)

byte drivespeed1 = 30;   //For Speed Setting (Normal): set this to whatever speeds works for you. 0-stop, 127-full speed.
byte drivespeed2 = 127;   //For Speed Setting (Over Throttle): set this for when needing extra power. 0-stop, 127-full speed.

byte turnspeed = 50;      // the higher this number the faster it will spin in place, lower - the easier to control.
                         // Recommend beginner: 40 to 50, experienced: 50+, I like 75

byte domespeed = 100;    // If using a speed controller for the dome, sets the top speed
                         // Use a number up to 127

byte ramping = 1;        // Ramping- the lower this number the longer R2 will take to speedup or slow down,
                         // change this by increments of 1

byte joystickFootDeadZoneRange = 10;  // For controllers that centering problems, use the lowest number with no drift
byte joystickDomeDeadZoneRange = 10;  // For controllers that centering problems, use the lowest number with no drift

byte driveDeadBandRange = 15;     // Used to set the Sabertooth DeadZone for foot motors

int invertTurnDirection = -1;   //This may need to be set to 1 for some configurations

byte domeAutoSpeed = 70;     // Speed used when dome automation is active - Valid Values: 50 - 100
int time360DomeTurn = 2500;  // milliseconds for dome to complete 360 turn at domeAutoSpeed - Valid Values: 2000 - 8000 (2000 = 2 seconds)

#define SHADOW_DEBUG       //uncomment this for console DEBUG output
#define SHADOW_VERBOSE     //uncomment this for console VERBOSE output



// ---------------------------------------------------------------------------------------
//               SYSTEM VARIABLES - USER CONFIG SECTION COMPLETED
// ---------------------------------------------------------------------------------------

#if FOOT_CONTROLLER == 1
#include <Servo.h>
#endif

// ---------------------------------------------------------------------------------------
//                          Drive Controller Settings
// ---------------------------------------------------------------------------------------

int motorControllerBaudRate = 9600; // Set the baud rate for the Syren motor controller
                                    // for packetized options are: 2400, 9600, 19200 and 38400

int marcDuinoBaudRate = 9600; // Set the baud rate for the Syren motor controller
                                    
#define SYREN_ADDR         129      // Serial Address for Dome Syren
#define SABERTOOTH_ADDR    128      // Serial Address for Foot Sabertooth

#define leftFootPin 44  //connect this pin to motor controller for left foot (R/C mode)
#define rightFootPin 45 //connect this pin to motor controller for right foot (R/C mode)
#define invertLeft 0    //if your foot motor is turning the wrong direction for some reason, change this from 0 to 1
#define invertRight 0
int footX,footY;

#define ENABLE_UHS_DEBUGGING 1

// ---------------------------------------------------------------------------------------
//                          Variables
// ---------------------------------------------------------------------------------------

long previousDomeMillis = millis();
long previousFootMillis = millis();
long previousMarcDuinoMillis = millis();
long previousDomeToggleMillis = millis();
long previousSpeedToggleMillis = millis();
long currentMillis = millis();

int serialLatency = 25;   //This is a delay factor in ms to prevent queueing of the Serial data.
                          //25ms seems approprate for HardwareSerial, values of 50ms or larger are needed for Softare Emulation
                          
int marcDuinoButtonCounter = 0;
int speedToggleButtonCounter = 0;
int domeToggleButtonCounter = 0;

#if FOOT_CONTROLLER == 0
Sabertooth *ST=new Sabertooth(SABERTOOTH_ADDR, Serial2); //
#endif
Sabertooth *SyR=new Sabertooth(SYREN_ADDR, Serial2);

///////Setup for USB and Bluetooth Devices////////////////////////////
USB Usb;
BTD Btd(&Usb);
PS3BT *PS3NavFoot=new PS3BT(&Btd);
PS3BT *PS3NavDome=new PS3BT(&Btd);

//Used for PS3 Fault Detection
uint32_t msgLagTime = 0;
uint32_t lastMsgTime = 0;
uint32_t currentTime = 0;
uint32_t lastLoopTime = 0;
int badPS3Data = 0;
int badPS3DataDome = 0;

boolean firstMessage = true;
String output = "";

boolean isFootMotorStopped = true;
boolean isDomeMotorStopped = true;

boolean overSpeedSelected = false;

boolean isPS3NavigatonInitialized = false;
boolean isSecondaryPS3NavigatonInitialized = false;

boolean isStickEnabled = true;

boolean WaitingforReconnect = false;
boolean WaitingforReconnectDome = false;

boolean mainControllerConnected = false;
boolean domeControllerConnected = false;

// Dome Automation Variables
boolean domeAutomation = false;
int domeTurnDirection = 1;  // 1 = positive turn, -1 negative turn
float domeTargetPosition = 0; // (0 - 359) - degrees in a circle, 0 = home
unsigned long domeStopTurnTime = 0;    // millis() when next turn should stop
unsigned long domeStartTurnTime = 0;  // millis() when next turn should start
int domeStatus = 0;  // 0 = stopped, 1 = prepare to turn, 2 = turning

byte action = 0;
unsigned long DriveMillis = 0;

int footDriveSpeed = 0;

// We use independent ramping for each foot if we're on Q85/Neo Etc ...
// Also the speed at "stop" is 90, not 0, so we set the default to 90. (servo mid point)
int leftFootDriveSpeed, rightFootDriveSpeed = 90;

#if FOOT_CONTROLLER ==1
Servo leftFootSignal;
Servo rightFootSignal;
#endif
