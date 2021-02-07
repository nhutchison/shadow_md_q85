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
//                          Libraries
// ---------------------------------------------------------------------------------------
#include <PS3BT.h>
#include <usbhub.h>

// Satisfy IDE, which only needs to see the include statment in the ino.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif

#include <Sabertooth.h>

// ---------------------------------------------------------------------------------------
//                          Local Includes
// ---------------------------------------------------------------------------------------
#include "config.h"
#include "md_config.h"



// ---------------------------------------------------------------------------------------
//               SYSTEM VARIABLES - USER CONFIG SECTION COMPLETED
// ---------------------------------------------------------------------------------------



// =======================================================================================
//                          Main Program
// =======================================================================================


// =======================================================================================
//                          Initialize - Setup Function
// =======================================================================================
void setup()
{
    //Debug Serial for use with USB Debugging
    Serial.begin(115200);
    while (!Serial);
    
    if (Usb.Init() == -1)
    {
        Serial.print(F("\r\nOSC did not start"));
        while (1); //halt
    }
    
    Serial.print(F("\r\nBluetooth Library Started"));
    
    output.reserve(200); // Reserve 200 bytes for the output string

    //Setup for PS3
    PS3NavFoot->attachOnInit(onInitPS3NavFoot); // onInitPS3NavFoot is called upon a new connection
    PS3NavDome->attachOnInit(onInitPS3NavDome); 

    //Setup for Serial2:: Motor Controllers - Sabertooth (Feet) 
    Serial2.begin(motorControllerBaudRate);
    #if FOOT_CONTROLLER == 0
    ST->autobaud();          // Send the autobaud command to the Sabertooth controller(s).
    ST->setTimeout(10);      //DMB:  How low can we go for safety reasons?  multiples of 100ms
    ST->setDeadband(driveDeadBandRange);
    #elif FOOT_CONTROLLER == 1
    leftFootSignal.attach(leftFootPin);
    rightFootSignal.attach(rightFootPin);
      #ifdef INTERNAL_MIXING
      // This may work for internal mixing, but lets not do it if the Robotex is mixing for us.
      drivespeed1 = map(drivespeed1, 0, 127, 0, 90); //convert drivespeed values to something that will work for the Q85's
      drivespeed2 = map(drivespeed2, 0, 127, 0, 90);
      #endif
    #endif
    stopFeet();
    SyR->autobaud();
    SyR->setTimeout(20);      //DMB:  How low can we go for safety reasons?  multiples of 100ms
    SyR->stop(); 

    //Setup for Serial1:: MarcDuino Dome Control Board
    Serial1.begin(marcDuinoBaudRate); 
    
    //Setup for Serial3:: Optional MarcDuino Control Board for Body Panels
    Serial3.begin(marcDuinoBaudRate);
    
    randomSeed(analogRead(0));  // random number seed for dome automation   
}

// =======================================================================================
//           Main Program Loop - This is the recurring check loop for entire sketch
// =======================================================================================

void loop()
{   
    //Useful to enable with serial console when having controller issues.
    #ifdef TEST_CONROLLER
      testPS3Controller();
    #endif

    //LOOP through functions from highest to lowest priority.

    if ( !readUSB() )
    {
      //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
      printOutput();
      return;
    }
    
    footMotorDrive();
    domeDrive();
    marcDuinoDome();
    marcDuinoFoot();
    toggleSettings();
    printOutput();
    
    // If running a custom MarcDuino Panel Routine - Call Function
    if (runningCustRoutine)
    {
       custMarcDuinoPanel();     
    }
    
    // If dome automation is enabled - Call function
    if (domeAutomation && time360DomeTurn > 1999 && time360DomeTurn < 8001 && domeAutoSpeed > 49 && domeAutoSpeed < 101)  
    {
       autoDome(); 
    }   
}

// =======================================================================================
//           footDrive Motor Control Section
// =======================================================================================

#if FOOT_CONTROLLER == 1
int leftFoot,rightFoot; //will hold foot speed values (-100 to 100)
int prevLeftFoot,prevRightFoot; //will hold foot speed values (-100 to 100)
float LeftSpeed,RightSpeed;
void mixBHD(byte stickX, byte stickY, byte maxSpeed){  //maxDriveSpeed should be between 90 and 180
    // This is BigHappyDude's mixing function, for differential (tank) style drive using two motor controllers.
    // Takes a joysticks X and Y values, mixes using the diamind mix, and output a value 0-180 for left and right motors.     
    // 180,180 = both feet full speed forward.
    // 000,000 = both feet full speed reverse.
    // 180,000 = left foot full forward, right foot full reverse (spin droid clockwise)
    // 000,180 = left foot full reverse, right foot full forward (spin droid counter-clockwise)
    // 090,090 = no movement
    // for simplicity, we think of this diamond matrix as a range from -100 to +100 , then map the final values to servo range (0-180) at the end 
    //  Ramping and Speed mode applied on the droid.  

    
    // Deadzone Handling
    // To map this to a servo, 90 is centre, which equals 128 on the joystick!
    if ( abs(stickX-128) < joystickFootDeadZoneRange ) 
    {
      stickX = 128;
    }

    if ( abs(stickY-128) < joystickFootDeadZoneRange ) 
    {
      stickY = 128;
    }
    
    if((stickX != 128) || (stickY != 128)) {
      //  Map to easy grid -100 to 100 in both axis, including deadzones.
      int YDist = 0;  // set to 0 as a default value if no if used.
      int XDist = 0;
      if(stickY < 128){
       YDist = (map(stickY, 0, 128, 100, 1));           //  Map the up direction stick value to Drive speed
      } else if(stickY > 128){
       YDist = (map(stickY, 128, 255, -1, -100));       //  Map the down direction stick value to Drive speed
      }
      if(stickX < 128){
       XDist = (map(stickX, 0, 128, -100, -1));       //  Map the left direction stick value to Turn speed
      } else if(stickX > 128){
       XDist = (map(stickX, 128, 255, 1, 100));   //  Map the right direction stick value to Turn speed
      }
      /*
      if(stickY <= 113){
       YDist = (map(stickY, 0, 113, 100, 1));           //  Map the up direction stick value to Drive speed
      } else if(stickY >= 141){
       YDist = (map(stickY, 141, 255, -1, -100));       //  Map the down direction stick value to Drive speed
      }
      if(stickX <= 113){
       XDist = (map(stickX, 0, 113, -100, -1));       //  Map the left direction stick value to Turn speed
      } else if(stickX >= 141){
       XDist = (map(stickX, 141, 255, 1, 100));   //  Map the right direction stick value to Turn speed
      }
      */
      //  Constrain to Diamond values.  using 2 line equations and find the intersect, boiled down to the minimum
      //  This was the inspiration; https://github.com/declanshanaghy/JabberBot/raw/master/Docs/Using%20Diamond%20Coordinates%20to%20Power%20a%20Differential%20Drive.pdf 
      float TempYDist = YDist;
      float TempXDist = XDist;
      if (YDist>(XDist+100)) {  //  if outside top left.  equation of line is y=x+Max, so if y > x+Max then it is above line
        // OK, the first fun bit. :)  so for the 2 lines this is always true y = m1*x + b1 and y = m2*x - b2
        // y - y = m1*x + b1  - m2*x - b2  or 0 = (m1 - m2)*x + b1 - b2
        // We have y = x+100 and y = ((change in y)/Change in x))x
        // So:   x = -100/(1-(change in y)/Change in x)) and using y = x+100 we can find y with the new x
        // Not too bad when simplified. :P
        TempXDist = -100/(1-(TempYDist/TempXDist));
        TempYDist = TempXDist+100;
      } else if (YDist>(100-XDist)) {  //  if outside top right
        // repeat intesection for y = 100 - x
        TempXDist = -100/(-1-(TempYDist/TempXDist));
        TempYDist = -TempXDist+100;
      } else if (YDist<(-XDist-100)) {  //  if outside bottom left
        // repeat intesection for y = -x - 100
        TempXDist = 100/(-1-(TempYDist/TempXDist));
        TempYDist = -TempXDist-100;
      } else if (YDist<(XDist-100)) {  //  if outside bottom right
        // repeat intesection for y = x - 100
        TempXDist = 100/(1-(TempYDist/TempXDist));
        TempYDist = TempXDist-100;
      }
      //  all coordinates now in diamond. next translate to the diamond coordinates.
      //  for the left.  send ray to y = x + Max from coordinates along y = -x + b
      //  find for b, solve for coordinates and resut in y then scale using y = (y - max/2)*2
      LeftSpeed = ((TempXDist+TempYDist-100)/2)+100;
      LeftSpeed = (LeftSpeed-50)*2;
      //  for right send ray to y = -x + Max from coordinates along y = x + b find intersction coordinates and then use the Y vaule and scale.
      RightSpeed = ((TempYDist-TempXDist-100)/2)+100;
      RightSpeed = (RightSpeed-50)*2;
      // this all results in a -100 to 100 range of speeds, so shift to servo range...
      //  eg. for a maxDriveSpeed of 140, we'd need the value to map to between 40 and 140
      //  eg. for a maxDriveSpeed of 180, we'd need the value to map to between 0 and 180
      //leftFoot=map(LeftSpeed, -100, 100, (180-maxDriveSpeed), maxDriveSpeed);
      //rightFoot=map(RightSpeed, -100, 100, (180-maxDriveSpeed), maxDriveSpeed);

      int servoMin = 90 - maxSpeed;
      int servoMax = 90 + maxSpeed;

      #if invertLeft == 0
        // Forward on the Neo motor is clockwise on the servo.
        leftFoot=map(LeftSpeed, 100, -100, servoMax, servoMin );
      #else
        leftFoot=map(LeftSpeed, 100, -100, servoMin, servoMax );
      #endif
      #if invertRight == 0
        rightFoot=map(RightSpeed, 100, -100, servoMax, servoMin );
      #else
        rightFoot=map(RightSpeed, 100, -100, servoMin, servoMax );
      #endif
    } else {
      leftFoot=90;
      rightFoot=90;
    }
}
#endif

void stopFeet() {
  #if FOOT_CONTROLLER == 0
  ST->stop();
  #elif FOOT_CONTROLLER == 1
  leftFoot=90;
  rightFoot=90;
  //leftFootSignal.write(90);
  //rightFootSignal.write(90);
  // Use writeMicroseconds to avoid any potential PWM issues.
  leftFootSignal.writeMicroseconds(1500);
  rightFootSignal.writeMicroseconds(1500);
  #endif
  #ifdef SHADOW_VERBOSE      
   output += "\r\n***Foot Motor STOPPED***\r\n";
  #endif   
}

boolean ps3FootMotorDrive(PS3BT* myPS = PS3NavFoot)
{
  int stickSpeed = 0;
  int turnnum = 0;
  
  if (isPS3NavigatonInitialized)
  {    
      // Additional fault control.  Do NOT send additional commands to Sabertooth if no controllers have initialized.
      if (!isStickEnabled)
      {
            #ifdef SHADOW_VERBOSE
              if ( abs(myPS->getAnalogHat(LeftHatY)-128) > joystickFootDeadZoneRange)
              {
                output += "Drive Stick is disabled\r\n";
              }
            #endif

          if (!isFootMotorStopped)
          {
              stopFeet();
              isFootMotorStopped = true;
              footDriveSpeed = 0;
              
              #ifdef SHADOW_VERBOSE      
                  output += "\r\n***Foot Motor STOPPED***\r\n";
              #endif              
          }
          
          return false;

      } else if (!myPS->PS3NavigationConnected)
      {
        
          if (!isFootMotorStopped)
          {
              stopFeet();
              isFootMotorStopped = true;
              footDriveSpeed = 0;

              #ifdef SHADOW_VERBOSE      
                  output += "\r\n***Foot Motor STOPPED***\r\n";
              #endif              
          }
          
          return false;

          
      } else if (myPS->getButtonPress(L2) || myPS->getButtonPress(L1))
      {
        
          if (!isFootMotorStopped)
          {
              stopFeet();
              isFootMotorStopped = true;
              footDriveSpeed = 0;

              #ifdef SHADOW_VERBOSE      
                  output += "\r\n***Foot Motor STOPPED***\r\n";
              #endif
              
          }
          
          return false;
        
      } else
      {
          //make those feet move!!!///////////////////////////////////////////////////
          int joystickPosition = myPS->getAnalogHat(LeftHatY);

         #if FOOT_CONTROLLER == 0
          if (overSpeedSelected) //Over throttle is selected
          {

            stickSpeed = (map(joystickPosition, 0, 255, -drivespeed2, drivespeed2));   
            
          } else 
          {
            
            stickSpeed = (map(joystickPosition, 0, 255, -drivespeed1, drivespeed1));
            
          }          

          if ( abs(joystickPosition-128) < joystickFootDeadZoneRange)
          {
  
                // This is RAMP DOWN code when stick is now at ZERO but prior FootSpeed > 20
                
                if (abs(footDriveSpeed) > 50)
                {   
                    if (footDriveSpeed > 0)
                    {
                        footDriveSpeed -= 3;
                    } else
                    {
                        footDriveSpeed += 3;
                    }
                    
                    #ifdef SHADOW_VERBOSE      
                        output += "ZERO FAST RAMP: footSpeed: ";
                        output += footDriveSpeed;
                        output += "\nStick Speed: ";
                        output += stickSpeed;
                        output += "\n\r";
                    #endif
                    
                } else if (abs(footDriveSpeed) > 20)
                {   
                    if (footDriveSpeed > 0)
                    {
                        footDriveSpeed -= 2;
                    } else
                    {
                        footDriveSpeed += 2;
                    }
                    
                    #ifdef SHADOW_VERBOSE      
                        output += "ZERO MID RAMP: footSpeed: ";
                        output += footDriveSpeed;
                        output += "\nStick Speed: ";
                        output += stickSpeed;
                        output += "\n\r";
                    #endif
                    
                } else
                {        
                    footDriveSpeed = 0;
                }
              
          } else 
          {
      
              isFootMotorStopped = false;
              
              if (footDriveSpeed < stickSpeed)
              {
                
                  if ((stickSpeed-footDriveSpeed)>(ramping+1))
                  {
                    footDriveSpeed+=ramping;
                      
                    #ifdef SHADOW_VERBOSE      
                        output += "RAMPING UP: footSpeed: ";
                        output += footDriveSpeed;
                        output += "\nStick Speed: ";
                        output += stickSpeed;
                        output += "\n\r";
                    #endif
                      
                  } else
                      footDriveSpeed = stickSpeed;
                  
              } else if (footDriveSpeed > stickSpeed)
              {
            
                  if ((footDriveSpeed-stickSpeed)>(ramping+1))
                  {
                    
                    footDriveSpeed-=ramping;
                      
                    #ifdef SHADOW_VERBOSE      
                        output += "RAMPING DOWN: footSpeed: ";
                        output += footDriveSpeed;
                        output += "\nStick Speed: ";
                        output += stickSpeed;
                        output += "\n\r";
                    #endif
                    
                  } else
                      footDriveSpeed = stickSpeed;  
              } else
              {
                  footDriveSpeed = stickSpeed;  
              }
          }
          
          turnnum = (myPS->getAnalogHat(LeftHatX));

          //TODO:  Is there a better algorithm here?  
          if ( abs(footDriveSpeed) > 50)
              turnnum = (map(myPS->getAnalogHat(LeftHatX), 54, 200, -(turnspeed/4), (turnspeed/4)));
          else if (turnnum <= 200 && turnnum >= 54)
              turnnum = (map(myPS->getAnalogHat(LeftHatX), 54, 200, -(turnspeed/3), (turnspeed/3)));
          else if (turnnum > 200)
              turnnum = (map(myPS->getAnalogHat(LeftHatX), 201, 255, turnspeed/3, turnspeed));
          else if (turnnum < 54)
              turnnum = (map(myPS->getAnalogHat(LeftHatX), 0, 53, -turnspeed, -(turnspeed/3)));
              
          if (abs(turnnum) > 5)
          {
              isFootMotorStopped = false;   
          }
         #endif

          currentMillis = millis();
          
          if ( (currentMillis - previousFootMillis) > serialLatency  )
          {

              #if FOOT_CONTROLLER == 0
              if (footDriveSpeed != 0 || abs(turnnum) > 5)
              #elif FOOT_CONTROLLER == 1
              //if (footDriveSpeed != 0) //footDriveSpeed doesn't change with the Q85's
              if (true)
              #endif
              {
                
                  #ifdef SHADOW_VERBOSE   
                    #if FOOT_CONTROLLER == 0   
                    output += "Motor: FootSpeed: ";
                    output += footDriveSpeed;
                    output += "\nTurnnum: ";              
                    output += turnnum;
                    output += "\nTime of command: ";              
                    output += millis();
                    #endif
                  #endif

                  #if FOOT_CONTROLLER == 0
                    ST->turn(turnnum * invertTurnDirection);
                    ST->drive(footDriveSpeed);
                  #elif FOOT_CONTROLLER == 1
                    isFootMotorStopped = false;
                    footX=myPS->getAnalogHat(LeftHatX);
                    footY=myPS->getAnalogHat(LeftHatY);
                    //Experimental Q85. Untested Madness!!! Use at your own risk and expect your droid to run away in flames.
                    //use BigHappyDude's mixing algorythm to get values for each foot...

                    byte maxDriveSpeed;
                    // Disable Internal BHD Mixing as the Roboteq does this for us.
                    #ifdef INTERNAL_MIXING

                    if (overSpeedSelected) {
                      mixBHD(footX,footY,drivespeed2);
                      maxDriveSpeed = drivespeed2;
                    }
                    else {
                      mixBHD(footX,footY,drivespeed1);
                      maxDriveSpeed = drivespeed1;                 
                    }
                    
                    #else
                      maxDriveSpeed = drivespeed1;
                      // Set the max speed of the drive motors
                      if (overSpeedSelected) {
                        maxDriveSpeed = drivespeed2;
                      }
                      // Neil's guess as to what to do here!

                      // Deadzone Handling
                      // To map this to a servo, 90 is centre, which equals 128 on the joystick!
                      if ( abs(footX-128) < joystickFootDeadZoneRange ) 
                      {
                        footX = 128;
                      }

                      if ( abs(footY-128) < joystickFootDeadZoneRange ) 
                      {
                        footY = 128;
                      }

                      // Map the foot into servo range
                      footX = map(footX, 0, 255, 0, 180 );
                      footY = map(footY, 0, 255, 0, 180 );

                      //Setup the endpoints for the servos
                      // Centred servo is 90, so min is 90- and max is 90+
                      // This means max speed is not 127, but 90 (90+90=180)
                      // Just check this in case someone has set too high a value!
                      if (maxDriveSpeed > 90)
                      {
                        maxDriveSpeed = 90;
                      }
                      // drive speed is mapped in Setup from 0-127 into 90-180 range
                      int servoMin = 90 - maxDriveSpeed;
                      int servoMax = 90 + maxDriveSpeed;
                    

                      leftFoot=map(footX, 0, 180, servoMin, servoMax );
                      rightFoot=map(footY, 0, 180, servoMin, servoMax );

                    #endif
                    
                    #ifdef SHADOW_VERBOSE   
                      if (prevLeftFoot!=leftFoot || prevRightFoot!=rightFoot) {
                        output += "  footX="+String(footX)+", footY="+String(footY);
                        output += "\n  LeftSpeed="+String(LeftSpeed)+", RightSpeed="+String(RightSpeed);
                        if (overSpeedSelected) output += "\ndrivespeed2";
                        else output += "\ndrivespeed1: ";
                        output += maxDriveSpeed;
                        output += "  L: ";
                        output += leftFoot;
                        output += "  R: ";              
                        output += rightFoot;

                      }
                    #endif

                      //domeRotationSpeed = (map(joystickPosition, 0, 255, -domespeed, domespeed));
                      
                      //now we've got values for leftFoot and rightFoot, output those somehow...
                      if (prevLeftFoot!=leftFoot || prevRightFoot!=rightFoot) {
                        // This converts the servo angle (0-180) into us values
                        int LeftFootus=map(leftFoot, 0, 180, 1000, 2000);
                        int RightFootus=map(rightFoot, 0, 180, 1000, 2000);
                        // Now send the us values
                        leftFootSignal.writeMicroseconds(LeftFootus);
                        rightFootSignal.writeMicroseconds(RightFootus);
                      }
                      
                    prevLeftFoot=leftFoot;
                    prevRightFoot=rightFoot;
                  #endif
              } else
              {    
                  if (!isFootMotorStopped)
                  {
                      stopFeet();
                      isFootMotorStopped = true;
                      footDriveSpeed = 0;
                      
                      #ifdef SHADOW_VERBOSE      
                         output += "\r\n***Foot Motor STOPPED***\r\n";
                      #endif
                  }              
              }
              
              previousFootMillis = currentMillis;
              return true; //we sent a foot command   
          }
      }
      //prevThrottle=myPS->getAnalogHat(LeftHatY); //used for blinkies
  }
  return false;
}

void footMotorDrive()
{

  //Flood control prevention
  if ((millis() - previousFootMillis) < serialLatency) return;  
  
  if (PS3NavFoot->PS3NavigationConnected) ps3FootMotorDrive(PS3NavFoot);
  
}  


// =======================================================================================
//           domeDrive Motor Control Section
// =======================================================================================

int ps3DomeDrive(PS3BT* myPS = PS3NavDome)
{
    int domeRotationSpeed = 0;
      
    int joystickPosition = myPS->getAnalogHat(LeftHatX);
        
    domeRotationSpeed = (map(joystickPosition, 0, 255, -domespeed, domespeed));
        
    if ( abs(joystickPosition-128) < joystickDomeDeadZoneRange ) 
       domeRotationSpeed = 0;
          
    if (domeRotationSpeed != 0 && domeAutomation == true)  // Turn off dome automation if manually moved
    {   
            domeAutomation = false; 
            domeStatus = 0;
            domeTargetPosition = 0; 
            
            #ifdef SHADOW_VERBOSE
              output += "Dome Automation OFF\r\n";
            #endif

    }    
    
    return domeRotationSpeed;
}

void rotateDome(int domeRotationSpeed, String mesg)
{
    //Constantly sending commands to the SyRen (Dome) is causing foot motor delay.
    //Lets reduce that chatter by trying 3 things:
    // 1.) Eliminate a constant stream of "don't spin" messages (isDomeMotorStopped flag)
    // 2.) Add a delay between commands sent to the SyRen (previousDomeMillis timer)
    // 3.) Switch to real UART on the MEGA (Likely the *CORE* issue and solution)
    // 4.) Reduce the timout of the SyRen - just better for safety!
    
    currentMillis = millis();
    if ( (!isDomeMotorStopped || domeRotationSpeed != 0) && ((currentMillis - previousDomeMillis) > (2*serialLatency) )  )
    {
      
          if (domeRotationSpeed != 0)
          {
            
            isDomeMotorStopped = false;
            
            #ifdef SHADOW_VERBOSE      
                output += "Dome rotation speed: ";
                output += domeRotationSpeed;
            #endif
        
            SyR->motor(domeRotationSpeed);
            
          } else
          {
            isDomeMotorStopped = true; 
            
            #ifdef SHADOW_VERBOSE      
                output += "\n\r***Dome motor is STOPPED***\n\r";
            #endif
            
            SyR->stop();
          }
          
          previousDomeMillis = currentMillis;      
    }
}

void domeDrive()
{
  //Flood control prevention
  //This is intentionally set to double the rate of the Dome Motor Latency
  if ((millis() - previousDomeMillis) < (2*serialLatency) ) return;  
  
  int domeRotationSpeed = 0;
  int ps3NavControlSpeed = 0;
  
  if (PS3NavDome->PS3NavigationConnected) 
  {
    
     ps3NavControlSpeed = ps3DomeDrive(PS3NavDome);

     domeRotationSpeed = ps3NavControlSpeed; 

     rotateDome(domeRotationSpeed,"Controller Move");
    
  } else if (PS3NavFoot->PS3NavigationConnected && PS3NavFoot->getButtonPress(L2))
  {
    
     ps3NavControlSpeed = ps3DomeDrive(PS3NavFoot);

     domeRotationSpeed = ps3NavControlSpeed; 

     rotateDome(domeRotationSpeed,"Controller Move");
    
  } else
  {
     if (!isDomeMotorStopped)
     {
         SyR->stop();
         isDomeMotorStopped = true;
     }
  }  
}  

// =======================================================================================
//                               Toggle Control Section
// =======================================================================================

void ps3ToggleSettings(PS3BT* myPS = PS3NavFoot)
{

    // enable / disable drive stick
    if(myPS->getButtonPress(PS) && myPS->getButtonClick(CROSS))
    {

        #ifdef SHADOW_DEBUG
          output += "Disabling the DriveStick\r\n";
          output += "Stopping Motors";
        #endif
        
        stopFeet();
        isFootMotorStopped = true;
        isStickEnabled = false;
        footDriveSpeed = 0;
    }
    
    if(myPS->getButtonPress(PS) && myPS->getButtonClick(CIRCLE))
    {
        #ifdef SHADOW_DEBUG
          output += "Enabling the DriveStick\r\n";
        #endif
        isStickEnabled = true;
    }
    
    // Enable and Disable Overspeed
    if (myPS->getButtonPress(L3) && myPS->getButtonPress(L1) && isStickEnabled)
    {
      
       if ((millis() - previousSpeedToggleMillis) > 1000)
       {
            speedToggleButtonCounter = 0;
            previousSpeedToggleMillis = millis();
       } 
     
       speedToggleButtonCounter += 1;
       
       if (speedToggleButtonCounter == 1)
       {
       
          if (!overSpeedSelected)
          {
           
                overSpeedSelected = true;
           
                #ifdef SHADOW_VERBOSE      
                  output += "Over Speed is now: ON";
                #endif
                
          } else
          {      
                overSpeedSelected = false;
           
                #ifdef SHADOW_VERBOSE      
                  output += "Over Speed is now: OFF";
                #endif   
          }  
       }
    }
   
    // Enable Disable Dome Automation
    if(myPS->getButtonPress(L2) && myPS->getButtonClick(CROSS))
    {
          domeAutomation = false;
          domeStatus = 0;
          domeTargetPosition = 0;
          SyR->stop();
          isDomeMotorStopped = true;
          
          #ifdef SHADOW_DEBUG
            output += "Dome Automation OFF\r\n";
          #endif
    } 

    if(myPS->getButtonPress(L2) && myPS->getButtonClick(CIRCLE))
    {
          domeAutomation = true;

          #ifdef SHADOW_DEBUG
            output += "Dome Automation On\r\n";
          #endif
    } 

}

void toggleSettings()
{
   if (PS3NavFoot->PS3NavigationConnected) ps3ToggleSettings(PS3NavFoot);
}  

// =======================================================================================
// This is the main MarcDuino Button Management Function
// =======================================================================================
void marcDuinoButtonPush(int type, int MD_func, int MP3_num, int LD_type, String LD_text, int panel_type, 
                         boolean use_DP1,
                         int DP1_str_delay, 
                         int DP1_open_time,
                         boolean use_DP2,
                         int DP2_str_delay, 
                         int DP2_open_time,
                         boolean use_DP3,
                         int DP3_str_delay, 
                         int DP3_open_time,
                         boolean use_DP4,
                         int DP4_str_delay, 
                         int DP4_open_time,
                         boolean use_DP5,
                         int DP5_str_delay, 
                         int DP5_open_time,
                         boolean use_DP6,
                         int DP6_str_delay, 
                         int DP6_open_time,
                         boolean use_DP7,
                         int DP7_str_delay, 
                         int DP7_open_time,
                         boolean use_DP8,
                         int DP8_str_delay, 
                         int DP8_open_time,
                         boolean use_DP9,
                         int DP9_str_delay, 
                         int DP9_open_time,
                         boolean use_DP10,
                         int DP10_str_delay, 
                         int DP10_open_time)
{
  
  if (type == 1)  // Std Marcduino Function Call Configured
  {
    
    switch (MD_func)
    {
      case 1:   
        Serial1.print(":SE00\r");  
        break;

      case 2:
        Serial1.print(":SE01\r");
        break;
        
      case 3:
        Serial1.print(":SE02\r");
        break;
        
      case 4:
        Serial1.print(":SE03\r");
        break;
                
      case 5:
        Serial1.print(":SE04\r");
        break;
                
      case 6:
        Serial1.print(":SE05\r");
        break;
                
      case 7:
        Serial1.print(":SE06\r");
        break;
                
      case 8:
        Serial1.print(":SE07\r");
        break;
                
      case 9:
        Serial1.print(":SE08\r");
        break;
                
      case 10:
        Serial1.print(":SE09\r");
        break;
                
      case 11:
        Serial1.print(":SE10\r");
        break;
                
      case 12:
        Serial1.print(":SE11\r");
        break;
                
      case 13:
        Serial1.print(":SE13\r");
        break;
                
      case 14:
        Serial1.print(":SE14\r");
        break;
                
      case 15:
        Serial1.print(":SE51\r");
        break;
                
      case 16:
        Serial1.print(":SE52\r");
        break;
                
      case 17:
        Serial1.print(":SE53\r");
        break;
                
      case 18:
        Serial1.print(":SE54\r");
        break;
                
      case 19:
        Serial1.print(":SE55\r");
        break;
                
      case 20:
        Serial1.print(":SE56\r");
        break;
                
      case 21:
        Serial1.print(":SE57\r");
        break;
                
      case 22:
        Serial1.print("*RD00\r");
        break;
                
      case 23:
        Serial1.print("*ON00\r");
        break;
                
      case 24:
        Serial1.print("*OF00\r");
        break;
                
      case 25:
        Serial1.print("*ST00\r");
        break;
                
      case 26:
        Serial1.print("$+\r");
        break;
                
      case 27:
        Serial1.print("$-\r");
        break;
                
      case 28:
        Serial1.print("$f\r");
        break;
                
      case 29:
        Serial1.print("$m\r");
        break;
                
      case 30:
        Serial1.print(":OP00\r");
        break;
                
      case 31:
        Serial1.print(":OP11\r");
        break;
                
      case 32:
        Serial1.print(":OP12\r");
        break;
                
      case 33:
        Serial1.print(":CL00\r");
        break;
                
      case 34:
        Serial1.print(":OP01\r");
        break;
                
      case 35:
        Serial1.print(":CL01\r");
        break;
                
      case 36:
        Serial1.print(":OP02\r");
        break;
                
      case 37:
        Serial1.print(":CL02\r");
        break;
                
      case 38:
        Serial1.print(":OP03\r");
        break;
                
      case 39:
        Serial1.print(":CL03\r");
        break;
                
      case 40:
        Serial1.print(":OP04\r");
        break;
                
      case 41:
        Serial1.print(":CL04\r");
        break;
                
      case 42:
        Serial1.print(":OP05\r");
        break;
                
      case 43:
        Serial1.print(":CL05\r");
        break;
                
      case 44:
        Serial1.print(":OP06\r");
        break;
                
      case 45:
        Serial1.print(":CL06\r");
        break;
                
      case 46:
        Serial1.print(":OP07\r");
        break;
                
      case 47:
        Serial1.print(":CL07\r");
        break;
                
      case 48:
        Serial1.print(":OP08\r");
        break;
                
      case 49:
        Serial1.print(":CL08\r");
        break;
                
      case 50:
        Serial1.print(":OP09\r");
        break;
                
      case 51:
        Serial1.print(":CL09\r");
        break;
                
      case 52:
        Serial1.print(":OP10\r");
        break;
                
      case 53:
        Serial1.print(":CL10\r");
        break;
                
      case 54:
        Serial3.print(":OP00\r");
        break;
                
      case 55:
        Serial3.print(":CL00\r");
        break;
                
      case 56:
        Serial3.print(":OP01\r");
        break;
                
      case 57:
        Serial3.print(":CL01\r");
        break;
                
      case 58:
        Serial3.print(":OP02\r");
        break;
                
      case 59:
        Serial3.print(":CL02\r");
        break;
                
      case 60:
        Serial3.print(":OP03\r");
        break;
                
      case 61:
        Serial3.print(":CL03\r");
        break;
                
      case 62:
        Serial3.print(":OP04\r");
        break;
                
      case 63:
        Serial3.print(":CL04\r");
        break;
                
      case 64:
        Serial3.print(":OP05\r");
        break;
                
      case 65:
        Serial3.print(":CL05\r");
        break;
                
      case 66:
        Serial3.print(":OP06\r");
        break;
                
      case 67:
        Serial3.print(":CL06\r");
        break;
                
      case 68:
        Serial3.print(":OP07\r");
        break;
                
      case 69:
        Serial3.print(":CL07\r");
        break;
                
      case 70:
        Serial3.print(":OP08\r");
        break;
                
      case 71:
        Serial3.print(":CL08\r");
        break;
                
      case 72:
        Serial3.print(":OP09\r");
        break;
                
      case 73:
        Serial3.print(":CL09\r");
        break;
                
      case 74:
        Serial3.print(":OP10\r");
        break;

      case 75:
        Serial3.print(":CL10\r");
        break;

      case 76:
        Serial3.print("*MO99\r");
        break;

      case 77:
        Serial3.print("*MO00\r");
        break;

      case 78:
        Serial3.print("*MF10\r");
        break;

    }  
    
  }  // End Std Marcduino Function Calls
   
   
  if (type == 2) // Custom Button Configuration
  {
   
      if (MP3_num > 181 && MP3_num < 201) // Valid Custom Sound Range Selected - Play Custom Sound Selection
      {
        
        switch (MP3_num)
        {
          
          case 182:
             Serial1.print("$87\r");
             break;
             
          case 183:
             Serial1.print("$88\r");
             break;
          
          case 184:
             Serial1.print("$89\r");
             break;

          case 185:
             Serial1.print("$810\r");
             break;
             
          case 186:
             Serial1.print("$811\r");
             break;
          
          case 187:
             Serial1.print("$812\r");
             break;
          case 188:
             Serial1.print("$813\r");
             break;
             
          case 189:
             Serial1.print("$814\r");
             break;
          
          case 190:
             Serial1.print("$815\r");
             break;
             
          case 191:
             Serial1.print("$816\r");
             break;
             
          case 192:
             Serial1.print("$817\r");
             break;
          
          case 193:
             Serial1.print("$818\r");
             break;
             
          case 194:
             Serial1.print("$819\r");
             break;
             
          case 195:
             Serial1.print("$820\r");
             break;
          
          case 196:
             Serial1.print("$821\r");
             break;
             
          case 197:
             Serial1.print("$822\r");
             break;
             
          case 198:
             Serial1.print("$823\r");
             break;
          
          case 199:
             Serial1.print("$824\r");
             break;

          case 200:
             Serial1.print("$825\r");
             break;
          
        }     
        
      }
      
      if (panel_type > 0 && panel_type < 10) // Valid panel type selected - perform custom panel functions
      {
        
          // Reset the custom panel flags
          DP1_Status = 0;
          DP2_Status = 0;
          DP3_Status = 0;
          DP4_Status = 0;
          DP5_Status = 0;
          DP6_Status = 0;
          DP7_Status = 0;
          DP8_Status = 0;
          DP9_Status = 0;
          DP10_Status = 0;
        
          if (panel_type > 1)
          {
            Serial1.print(":CL00\r");  // close all the panels prior to next custom routine
            delay(50); // give panel close command time to process before starting next panel command 
          }
        
          switch (panel_type)
          {
            
             case 1:
                Serial1.print(":CL00\r");
                break;
                
             case 2:
                Serial1.print(":SE51\r");
                break;
                
             case 3:
                Serial1.print(":SE52\r");
                break;

             case 4:
                Serial1.print(":SE53\r");
                break;

             case 5:
                Serial1.print(":SE54\r");
                break;

             case 6:
                Serial1.print(":SE55\r");
                break;

             case 7:
                Serial1.print(":SE56\r");
                break;

             case 8:
                Serial1.print(":SE57\r");
                break;

             case 9: // This is the setup section for the custom panel routines
             
                runningCustRoutine = true;
                
                // Configure Dome Panel #1
                if (use_DP1)
                {
                  
                  DP1_Status = 1; 
                  DP1_start = millis();
                  
                  if (DP1_str_delay < 31)
                  {
                    
                       DP1_s_delay = DP1_str_delay; 
                       
                  } else
                  {
                       DP1_Status = 0; 
                  }
                  
                  if (DP1_open_time > 0 && DP1_open_time < 31)
                  {
                    
                       DP1_o_time = DP1_open_time; 
                       
                  } else
                  {
                       DP1_Status = 0; 
                  }
                      
                }
                
                // Configure Dome Panel #2
                if (use_DP2)
                {
                  
                  DP2_Status = 1; 
                  DP2_start = millis();
                  
                  if (DP2_str_delay < 31)
                  {
                    
                       DP2_s_delay = DP2_str_delay; 
                       
                  } else
                  {
                       DP2_Status = 0; 
                  }
                  
                  if (DP2_open_time > 0 && DP2_open_time < 31)
                  {
                    
                       DP2_o_time = DP2_open_time; 
                       
                  } else
                  {
                       DP2_Status = 0; 
                  } 
       
                }
                

                // Configure Dome Panel #3
                if (use_DP3)
                {
                  
                  DP3_Status = 1; 
                  DP3_start = millis();
                  
                  if (DP3_str_delay < 31)
                  {
                    
                       DP3_s_delay = DP3_str_delay; 
                       
                  } else
                  {
                       DP3_Status = 0; 
                  }
                  
                  if (DP3_open_time > 0 && DP3_open_time < 31)
                  {
                    
                       DP3_o_time = DP3_open_time; 
                       
                  } else
                  {
                       DP3_Status = 0; 
                  } 
       
                }
                
                // Configure Dome Panel #4
                if (use_DP4)
                {
                  
                  DP4_Status = 1; 
                  DP4_start = millis();
                  
                  if (DP4_str_delay < 31)
                  {
                    
                       DP4_s_delay = DP4_str_delay; 
                       
                  } else
                  {
                       DP4_Status = 0; 
                  }
                  
                  if (DP4_open_time > 0 && DP4_open_time < 31)
                  {
                    
                       DP4_o_time = DP4_open_time; 
                       
                  } else
                  {
                       DP4_Status = 0; 
                  } 
       
                }
                
                // Configure Dome Panel #5
                if (use_DP5)
                {
                  
                  DP5_Status = 1; 
                  DP5_start = millis();
                  
                  if (DP5_str_delay < 31)
                  {
                    
                       DP5_s_delay = DP5_str_delay; 
                       
                  } else
                  {
                       DP5_Status = 0; 
                  }
                  
                  if (DP5_open_time > 0 && DP5_open_time < 31)
                  {
                    
                       DP5_o_time = DP5_open_time; 
                       
                  } else
                  {
                       DP5_Status = 0; 
                  } 
       
                }
                
                // Configure Dome Panel #6
                if (use_DP6)
                {
                  
                  DP6_Status = 1; 
                  DP6_start = millis();
                  
                  if (DP6_str_delay < 31)
                  {
                    
                       DP6_s_delay = DP6_str_delay; 
                       
                  } else
                  {
                       DP6_Status = 0; 
                  }
                  
                  if (DP6_open_time > 0 && DP6_open_time < 31)
                  {
                    
                       DP6_o_time = DP6_open_time; 
                       
                  } else
                  {
                       DP6_Status = 0; 
                  } 
       
                }
                
                // Configure Dome Panel #7
                if (use_DP7)
                {
                  
                  DP7_Status = 1; 
                  DP7_start = millis();
                  
                  if (DP7_str_delay < 31)
                  {
                    
                       DP7_s_delay = DP7_str_delay; 
                       
                  } else
                  {
                       DP7_Status = 0; 
                  }
                  
                  if (DP7_open_time > 0 && DP7_open_time < 31)
                  {
                    
                       DP7_o_time = DP7_open_time; 
                       
                  } else
                  {
                       DP7_Status = 0; 
                  } 
       
                }
                
                // Configure Dome Panel #8
                if (use_DP8)
                {
                  
                  DP8_Status = 1; 
                  DP8_start = millis();
                  
                  if (DP8_str_delay < 31)
                  {
                    
                       DP8_s_delay = DP8_str_delay; 
                       
                  } else
                  {
                       DP8_Status = 0; 
                  }
                  
                  if (DP8_open_time > 0 && DP8_open_time < 31)
                  {
                    
                       DP8_o_time = DP8_open_time; 
                       
                  } else
                  {
                       DP8_Status = 0; 
                  } 
       
                }
                
                // Configure Dome Panel #9
                if (use_DP9)
                {
                  
                  DP9_Status = 1; 
                  DP9_start = millis();
                  
                  if (DP9_str_delay < 31)
                  {
                    
                       DP9_s_delay = DP9_str_delay; 
                       
                  } else
                  {
                       DP9_Status = 0; 
                  }
                  
                  if (DP9_open_time > 0 && DP9_open_time < 31)
                  {
                    
                       DP9_o_time = DP9_open_time; 
                       
                  } else
                  {
                       DP9_Status = 0; 
                  } 
       
                }
                
                // Configure Dome Panel #10
                if (use_DP10)
                {
                  
                  DP10_Status = 1; 
                  DP10_start = millis();
                  
                  if (DP10_str_delay < 31)
                  {
                    
                       DP10_s_delay = DP10_str_delay; 
                       
                  } else
                  {
                       DP10_Status = 0; 
                  }
                  
                  if (DP10_open_time > 0 && DP10_open_time < 31)
                  {
                    
                       DP10_o_time = DP10_open_time; 
                       
                  } else
                  {
                       DP10_Status = 0; 
                  } 
       
                }
                              
                // If every dome panel config failed to work - reset routine flag to false
                if (DP1_Status + DP2_Status + DP3_Status + DP4_Status + DP5_Status + DP6_Status + DP7_Status + DP8_Status + DP9_Status + DP10_Status == 0)
                {
                   
                   runningCustRoutine = false;

                }
                
                break;           
          }
      }
        
      
      if (LD_type > 0 && LD_type < 9) // Valid Logic Display Selected - Display Custom Logic Display
      {
        
          if (panel_type > 1 && panel_type < 10)  // If a custom panel movement was selected - need to briefly pause before changing light sequence to avoid conflict)
          {   
              delay(30);
          }
        
          switch (LD_type)
          {
            
            case 1:
              Serial1.print("@0T1\r");
              break;
              
            case 2:
              Serial1.print("@0T4\r");
              break;
              
            case 3:
              Serial1.print("@0T5\r");
              break;

            case 4:
              Serial1.print("@0T6\r");
              break;

            case 5:
              Serial1.print("@0T10\r");
              break;

            case 6:
              Serial1.print("@0T11\r");
              break;

            case 7:
              Serial1.print("@0T92\r");
              break;

            case 8:
              Serial1.print("@0T100\r");
              delay(50);
              String custString = "@0M";
              custString += LD_text;
              custString += "\r";
              Serial1.print(custString);
              break;
          }
      }
       
  } 
  
}

// ====================================================================================================================
// This function determines if MarcDuino buttons were selected and calls main processing function for FOOT controller
// ====================================================================================================================
void marcDuinoFoot()
{
   if (PS3NavFoot->PS3NavigationConnected && (PS3NavFoot->getButtonPress(UP) || PS3NavFoot->getButtonPress(DOWN) || PS3NavFoot->getButtonPress(LEFT) || PS3NavFoot->getButtonPress(RIGHT)))
   {
      
       if ((millis() - previousMarcDuinoMillis) > 1000)
       {
            marcDuinoButtonCounter = 0;
            previousMarcDuinoMillis = millis();
       } 
     
       marcDuinoButtonCounter += 1;
         
   } else
   {
       return;    
   }
   
   // Clear inbound buffer of any data sent form the MarcDuino board
   while (Serial1.available()) Serial1.read();

    //------------------------------------ 
    // Send triggers for the base buttons 
    //------------------------------------
    if (PS3NavFoot->getButtonPress(UP) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(L1) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
    {

       if (PS3NavDome->PS3NavigationConnected && (PS3NavDome->getButtonPress(CROSS) || PS3NavDome->getButtonPress(CIRCLE) || PS3NavDome->getButtonPress(PS)))
       {
              // Skip this section
       } else
       {     
               marcDuinoButtonPush(btnUP_type, btnUP_MD_func, btnUP_cust_MP3_num, btnUP_cust_LD_type, btnUP_cust_LD_text, btnUP_cust_panel, 
                                 btnUP_use_DP1,
                                 btnUP_DP1_open_start_delay, 
                                 btnUP_DP1_stay_open_time,
                                 btnUP_use_DP2,
                                 btnUP_DP2_open_start_delay, 
                                 btnUP_DP2_stay_open_time,
                                 btnUP_use_DP3,
                                 btnUP_DP3_open_start_delay, 
                                 btnUP_DP3_stay_open_time,
                                 btnUP_use_DP4,
                                 btnUP_DP4_open_start_delay, 
                                 btnUP_DP4_stay_open_time,
                                 btnUP_use_DP5,
                                 btnUP_DP5_open_start_delay, 
                                 btnUP_DP5_stay_open_time,
                                 btnUP_use_DP6,
                                 btnUP_DP6_open_start_delay, 
                                 btnUP_DP6_stay_open_time,
                                 btnUP_use_DP7,
                                 btnUP_DP7_open_start_delay, 
                                 btnUP_DP7_stay_open_time,
                                 btnUP_use_DP8,
                                 btnUP_DP8_open_start_delay, 
                                 btnUP_DP8_stay_open_time,
                                 btnUP_use_DP9,
                                 btnUP_DP9_open_start_delay, 
                                 btnUP_DP9_stay_open_time,
                                 btnUP_use_DP10,
                                 btnUP_DP10_open_start_delay, 
                                 btnUP_DP10_stay_open_time);
                    
                #ifdef SHADOW_VERBOSE      
                     output += "FOOT: btnUP";
                #endif
               
                return;
        
         }
        
    }
    
    if (PS3NavFoot->getButtonPress(DOWN) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(L1) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
    {
      
       if (PS3NavDome->PS3NavigationConnected && (PS3NavDome->getButtonPress(CROSS) || PS3NavDome->getButtonPress(CIRCLE) || PS3NavDome->getButtonPress(PS)))
       {
              // Skip this section
       } else
       {     
            marcDuinoButtonPush(btnDown_type, btnDown_MD_func, btnDown_cust_MP3_num, btnDown_cust_LD_type, btnDown_cust_LD_text, btnDown_cust_panel, 
                         btnDown_use_DP1,
                         btnDown_DP1_open_start_delay, 
                         btnDown_DP1_stay_open_time,
                         btnDown_use_DP2,
                         btnDown_DP2_open_start_delay, 
                         btnDown_DP2_stay_open_time,
                         btnDown_use_DP3,
                         btnDown_DP3_open_start_delay, 
                         btnDown_DP3_stay_open_time,
                         btnDown_use_DP4,
                         btnDown_DP4_open_start_delay, 
                         btnDown_DP4_stay_open_time,
                         btnDown_use_DP5,
                         btnDown_DP5_open_start_delay, 
                         btnDown_DP5_stay_open_time,
                         btnDown_use_DP6,
                         btnDown_DP6_open_start_delay, 
                         btnDown_DP6_stay_open_time,
                         btnDown_use_DP7,
                         btnDown_DP7_open_start_delay, 
                         btnDown_DP7_stay_open_time,
                         btnDown_use_DP8,
                         btnDown_DP8_open_start_delay, 
                         btnDown_DP8_stay_open_time,
                         btnDown_use_DP9,
                         btnDown_DP9_open_start_delay, 
                         btnDown_DP9_stay_open_time,
                         btnDown_use_DP10,
                         btnDown_DP10_open_start_delay, 
                         btnDown_DP10_stay_open_time);
                         
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnDown";
        #endif
      
       
        return;
       }
    }
    
    if (PS3NavFoot->getButtonPress(LEFT) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(L1) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
    {
       if (PS3NavDome->PS3NavigationConnected && (PS3NavDome->getButtonPress(CROSS) || PS3NavDome->getButtonPress(CIRCLE) || PS3NavDome->getButtonPress(PS)))
       {
              // Skip this section
       } else
       {           
            marcDuinoButtonPush(btnLeft_type, btnLeft_MD_func, btnLeft_cust_MP3_num, btnLeft_cust_LD_type, btnLeft_cust_LD_text, btnLeft_cust_panel, 
                         btnLeft_use_DP1,
                         btnLeft_DP1_open_start_delay, 
                         btnLeft_DP1_stay_open_time,
                         btnLeft_use_DP2,
                         btnLeft_DP2_open_start_delay, 
                         btnLeft_DP2_stay_open_time,
                         btnLeft_use_DP3,
                         btnLeft_DP3_open_start_delay, 
                         btnLeft_DP3_stay_open_time,
                         btnLeft_use_DP4,
                         btnLeft_DP4_open_start_delay, 
                         btnLeft_DP4_stay_open_time,
                         btnLeft_use_DP5,
                         btnLeft_DP5_open_start_delay, 
                         btnLeft_DP5_stay_open_time,
                         btnLeft_use_DP6,
                         btnLeft_DP6_open_start_delay, 
                         btnLeft_DP6_stay_open_time,
                         btnLeft_use_DP7,
                         btnLeft_DP7_open_start_delay, 
                         btnLeft_DP7_stay_open_time,
                         btnLeft_use_DP8,
                         btnLeft_DP8_open_start_delay, 
                         btnLeft_DP8_stay_open_time,
                         btnLeft_use_DP9,
                         btnLeft_DP9_open_start_delay, 
                         btnLeft_DP9_stay_open_time,
                         btnLeft_use_DP10,
                         btnLeft_DP10_open_start_delay, 
                         btnLeft_DP10_stay_open_time);
                         
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnLeft";
        #endif
       
        return;
       }
        
     }

    if (PS3NavFoot->getButtonPress(RIGHT) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(L1) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
    {
      
       if (PS3NavDome->PS3NavigationConnected && (PS3NavDome->getButtonPress(CROSS) || PS3NavDome->getButtonPress(CIRCLE) || PS3NavDome->getButtonPress(PS)))
       {
              // Skip this section
       } else
       {     
             marcDuinoButtonPush(btnRight_type, btnRight_MD_func, btnRight_cust_MP3_num, btnRight_cust_LD_type, btnRight_cust_LD_text, btnRight_cust_panel, 
                         btnRight_use_DP1,
                         btnRight_DP1_open_start_delay, 
                         btnRight_DP1_stay_open_time,
                         btnRight_use_DP2,
                         btnRight_DP2_open_start_delay, 
                         btnRight_DP2_stay_open_time,
                         btnRight_use_DP3,
                         btnRight_DP3_open_start_delay, 
                         btnRight_DP3_stay_open_time,
                         btnRight_use_DP4,
                         btnRight_DP4_open_start_delay, 
                         btnRight_DP4_stay_open_time,
                         btnRight_use_DP5,
                         btnRight_DP5_open_start_delay, 
                         btnRight_DP5_stay_open_time,
                         btnRight_use_DP6,
                         btnRight_DP6_open_start_delay, 
                         btnRight_DP6_stay_open_time,
                         btnRight_use_DP7,
                         btnRight_DP7_open_start_delay, 
                         btnRight_DP7_stay_open_time,
                         btnRight_use_DP8,
                         btnRight_DP8_open_start_delay, 
                         btnRight_DP8_stay_open_time,
                         btnRight_use_DP9,
                         btnRight_DP9_open_start_delay, 
                         btnRight_DP9_stay_open_time,
                         btnRight_use_DP10,
                         btnRight_DP10_open_start_delay, 
                         btnRight_DP10_stay_open_time);
                         
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnRight";
        #endif
      
       
        return;
       }
        
    }
    
    //------------------------------------ 
    // Send triggers for the CROSS + base buttons 
    //------------------------------------
    if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(UP) && PS3NavFoot->getButtonPress(CROSS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(UP) && PS3NavDome->getButtonPress(CROSS))) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnUP_CROSS_type, btnUP_CROSS_MD_func, btnUP_CROSS_cust_MP3_num, btnUP_CROSS_cust_LD_type, btnUP_CROSS_cust_LD_text, btnUP_CROSS_cust_panel, 
                         btnUP_CROSS_use_DP1,
                         btnUP_CROSS_DP1_open_start_delay, 
                         btnUP_CROSS_DP1_stay_open_time,
                         btnUP_CROSS_use_DP2,
                         btnUP_CROSS_DP2_open_start_delay, 
                         btnUP_CROSS_DP2_stay_open_time,
                         btnUP_CROSS_use_DP3,
                         btnUP_CROSS_DP3_open_start_delay, 
                         btnUP_CROSS_DP3_stay_open_time,
                         btnUP_CROSS_use_DP4,
                         btnUP_CROSS_DP4_open_start_delay, 
                         btnUP_CROSS_DP4_stay_open_time,
                         btnUP_CROSS_use_DP5,
                         btnUP_CROSS_DP5_open_start_delay, 
                         btnUP_CROSS_DP5_stay_open_time,
                         btnUP_CROSS_use_DP6,
                         btnUP_CROSS_DP6_open_start_delay, 
                         btnUP_CROSS_DP6_stay_open_time,
                         btnUP_CROSS_use_DP7,
                         btnUP_CROSS_DP7_open_start_delay, 
                         btnUP_CROSS_DP7_stay_open_time,
                         btnUP_CROSS_use_DP8,
                         btnUP_CROSS_DP8_open_start_delay, 
                         btnUP_CROSS_DP8_stay_open_time,
                         btnUP_CROSS_use_DP9,
                         btnUP_CROSS_DP9_open_start_delay, 
                         btnUP_CROSS_DP9_stay_open_time,
                         btnUP_CROSS_use_DP10,
                         btnUP_CROSS_DP10_open_start_delay, 
                         btnUP_CROSS_DP10_stay_open_time);
      
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnUP_CROSS";
        #endif
      
       
        return;
        
    }
    
    if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(DOWN) && PS3NavFoot->getButtonPress(CROSS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(DOWN) && PS3NavDome->getButtonPress(CROSS))) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnDown_CROSS_type, btnDown_CROSS_MD_func, btnDown_CROSS_cust_MP3_num, btnDown_CROSS_cust_LD_type, btnDown_CROSS_cust_LD_text, btnDown_CROSS_cust_panel, 
                         btnDown_CROSS_use_DP1,
                         btnDown_CROSS_DP1_open_start_delay, 
                         btnDown_CROSS_DP1_stay_open_time,
                         btnDown_CROSS_use_DP2,
                         btnDown_CROSS_DP2_open_start_delay, 
                         btnDown_CROSS_DP2_stay_open_time,
                         btnDown_CROSS_use_DP3,
                         btnDown_CROSS_DP3_open_start_delay, 
                         btnDown_CROSS_DP3_stay_open_time,
                         btnDown_CROSS_use_DP4,
                         btnDown_CROSS_DP4_open_start_delay, 
                         btnDown_CROSS_DP4_stay_open_time,
                         btnDown_CROSS_use_DP5,
                         btnDown_CROSS_DP5_open_start_delay, 
                         btnDown_CROSS_DP5_stay_open_time,
                         btnDown_CROSS_use_DP6,
                         btnDown_CROSS_DP6_open_start_delay, 
                         btnDown_CROSS_DP6_stay_open_time,
                         btnDown_CROSS_use_DP7,
                         btnDown_CROSS_DP7_open_start_delay, 
                         btnDown_CROSS_DP7_stay_open_time,
                         btnDown_CROSS_use_DP8,
                         btnDown_CROSS_DP8_open_start_delay, 
                         btnDown_CROSS_DP8_stay_open_time,
                         btnDown_CROSS_use_DP9,
                         btnDown_CROSS_DP9_open_start_delay, 
                         btnDown_CROSS_DP9_stay_open_time,
                         btnDown_CROSS_use_DP10,
                         btnDown_CROSS_DP10_open_start_delay, 
                         btnDown_CROSS_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnDown_CROSS";
        #endif
      
       
        return;
        
    }
    
    if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(LEFT) && PS3NavFoot->getButtonPress(CROSS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(LEFT) && PS3NavDome->getButtonPress(CROSS))) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnLeft_CROSS_type, btnLeft_CROSS_MD_func, btnLeft_CROSS_cust_MP3_num, btnLeft_CROSS_cust_LD_type, btnLeft_CROSS_cust_LD_text, btnLeft_CROSS_cust_panel, 
                         btnLeft_CROSS_use_DP1,
                         btnLeft_CROSS_DP1_open_start_delay, 
                         btnLeft_CROSS_DP1_stay_open_time,
                         btnLeft_CROSS_use_DP2,
                         btnLeft_CROSS_DP2_open_start_delay, 
                         btnLeft_CROSS_DP2_stay_open_time,
                         btnLeft_CROSS_use_DP3,
                         btnLeft_CROSS_DP3_open_start_delay, 
                         btnLeft_CROSS_DP3_stay_open_time,
                         btnLeft_CROSS_use_DP4,
                         btnLeft_CROSS_DP4_open_start_delay, 
                         btnLeft_CROSS_DP4_stay_open_time,
                         btnLeft_CROSS_use_DP5,
                         btnLeft_CROSS_DP5_open_start_delay, 
                         btnLeft_CROSS_DP5_stay_open_time,
                         btnLeft_CROSS_use_DP6,
                         btnLeft_CROSS_DP6_open_start_delay, 
                         btnLeft_CROSS_DP6_stay_open_time,
                         btnLeft_CROSS_use_DP7,
                         btnLeft_CROSS_DP7_open_start_delay, 
                         btnLeft_CROSS_DP7_stay_open_time,
                         btnLeft_CROSS_use_DP8,
                         btnLeft_CROSS_DP8_open_start_delay, 
                         btnLeft_CROSS_DP8_stay_open_time,
                         btnLeft_CROSS_use_DP9,
                         btnLeft_CROSS_DP9_open_start_delay, 
                         btnLeft_CROSS_DP9_stay_open_time,
                         btnLeft_CROSS_use_DP10,
                         btnLeft_CROSS_DP10_open_start_delay, 
                         btnLeft_CROSS_DP10_stay_open_time);
            
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnLeft_CROSS";
        #endif
      
       
        return;
        
    }

    if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(RIGHT) && PS3NavFoot->getButtonPress(CROSS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(RIGHT) && PS3NavDome->getButtonPress(CROSS))) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnRight_CROSS_type, btnRight_CROSS_MD_func, btnRight_CROSS_cust_MP3_num, btnRight_CROSS_cust_LD_type, btnRight_CROSS_cust_LD_text, btnRight_CROSS_cust_panel, 
                         btnRight_CROSS_use_DP1,
                         btnRight_CROSS_DP1_open_start_delay, 
                         btnRight_CROSS_DP1_stay_open_time,
                         btnRight_CROSS_use_DP2,
                         btnRight_CROSS_DP2_open_start_delay, 
                         btnRight_CROSS_DP2_stay_open_time,
                         btnRight_CROSS_use_DP3,
                         btnRight_CROSS_DP3_open_start_delay, 
                         btnRight_CROSS_DP3_stay_open_time,
                         btnRight_CROSS_use_DP4,
                         btnRight_CROSS_DP4_open_start_delay, 
                         btnRight_CROSS_DP4_stay_open_time,
                         btnRight_CROSS_use_DP5,
                         btnRight_CROSS_DP5_open_start_delay, 
                         btnRight_CROSS_DP5_stay_open_time,
                         btnRight_CROSS_use_DP6,
                         btnRight_CROSS_DP6_open_start_delay, 
                         btnRight_CROSS_DP6_stay_open_time,
                         btnRight_CROSS_use_DP7,
                         btnRight_CROSS_DP7_open_start_delay, 
                         btnRight_CROSS_DP7_stay_open_time,
                         btnRight_CROSS_use_DP8,
                         btnRight_CROSS_DP8_open_start_delay, 
                         btnRight_CROSS_DP8_stay_open_time,
                         btnRight_CROSS_use_DP9,
                         btnRight_CROSS_DP9_open_start_delay, 
                         btnRight_CROSS_DP9_stay_open_time,
                         btnRight_CROSS_use_DP10,
                         btnRight_CROSS_DP10_open_start_delay, 
                         btnRight_CROSS_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnRight_CROSS";
        #endif
      
       
        return;
        
    }

    //------------------------------------ 
    // Send triggers for the CIRCLE + base buttons 
    //------------------------------------
    if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(UP) && PS3NavFoot->getButtonPress(CIRCLE)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(UP) && PS3NavDome->getButtonPress(CIRCLE))) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnUP_CIRCLE_type, btnUP_CIRCLE_MD_func, btnUP_CIRCLE_cust_MP3_num, btnUP_CIRCLE_cust_LD_type, btnUP_CIRCLE_cust_LD_text, btnUP_CIRCLE_cust_panel, 
                         btnUP_CIRCLE_use_DP1,
                         btnUP_CIRCLE_DP1_open_start_delay, 
                         btnUP_CIRCLE_DP1_stay_open_time,
                         btnUP_CIRCLE_use_DP2,
                         btnUP_CIRCLE_DP2_open_start_delay, 
                         btnUP_CIRCLE_DP2_stay_open_time,
                         btnUP_CIRCLE_use_DP3,
                         btnUP_CIRCLE_DP3_open_start_delay, 
                         btnUP_CIRCLE_DP3_stay_open_time,
                         btnUP_CIRCLE_use_DP4,
                         btnUP_CIRCLE_DP4_open_start_delay, 
                         btnUP_CIRCLE_DP4_stay_open_time,
                         btnUP_CIRCLE_use_DP5,
                         btnUP_CIRCLE_DP5_open_start_delay, 
                         btnUP_CIRCLE_DP5_stay_open_time,
                         btnUP_CIRCLE_use_DP6,
                         btnUP_CIRCLE_DP6_open_start_delay, 
                         btnUP_CIRCLE_DP6_stay_open_time,
                         btnUP_CIRCLE_use_DP7,
                         btnUP_CIRCLE_DP7_open_start_delay, 
                         btnUP_CIRCLE_DP7_stay_open_time,
                         btnUP_CIRCLE_use_DP8,
                         btnUP_CIRCLE_DP8_open_start_delay, 
                         btnUP_CIRCLE_DP8_stay_open_time,
                         btnUP_CIRCLE_use_DP9,
                         btnUP_CIRCLE_DP9_open_start_delay, 
                         btnUP_CIRCLE_DP9_stay_open_time,
                         btnUP_CIRCLE_use_DP10,
                         btnUP_CIRCLE_DP10_open_start_delay, 
                         btnUP_CIRCLE_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnUP_CIRCLE";
        #endif
      
       
        return;
        
    }
    
    if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(DOWN) && PS3NavFoot->getButtonPress(CIRCLE)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(DOWN) && PS3NavDome->getButtonPress(CIRCLE))) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnDown_CIRCLE_type, btnDown_CIRCLE_MD_func, btnDown_CIRCLE_cust_MP3_num, btnDown_CIRCLE_cust_LD_type, btnDown_CIRCLE_cust_LD_text, btnDown_CIRCLE_cust_panel, 
                         btnDown_CIRCLE_use_DP1,
                         btnDown_CIRCLE_DP1_open_start_delay, 
                         btnDown_CIRCLE_DP1_stay_open_time,
                         btnDown_CIRCLE_use_DP2,
                         btnDown_CIRCLE_DP2_open_start_delay, 
                         btnDown_CIRCLE_DP2_stay_open_time,
                         btnDown_CIRCLE_use_DP3,
                         btnDown_CIRCLE_DP3_open_start_delay, 
                         btnDown_CIRCLE_DP3_stay_open_time,
                         btnDown_CIRCLE_use_DP4,
                         btnDown_CIRCLE_DP4_open_start_delay, 
                         btnDown_CIRCLE_DP4_stay_open_time,
                         btnDown_CIRCLE_use_DP5,
                         btnDown_CIRCLE_DP5_open_start_delay, 
                         btnDown_CIRCLE_DP5_stay_open_time,
                         btnDown_CIRCLE_use_DP6,
                         btnDown_CIRCLE_DP6_open_start_delay, 
                         btnDown_CIRCLE_DP6_stay_open_time,
                         btnDown_CIRCLE_use_DP7,
                         btnDown_CIRCLE_DP7_open_start_delay, 
                         btnDown_CIRCLE_DP7_stay_open_time,
                         btnDown_CIRCLE_use_DP8,
                         btnDown_CIRCLE_DP8_open_start_delay, 
                         btnDown_CIRCLE_DP8_stay_open_time,
                         btnDown_CIRCLE_use_DP9,
                         btnDown_CIRCLE_DP9_open_start_delay, 
                         btnDown_CIRCLE_DP9_stay_open_time,
                         btnDown_CIRCLE_use_DP10,
                         btnDown_CIRCLE_DP10_open_start_delay, 
                         btnDown_CIRCLE_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnDown_CIRCLE";
        #endif
      
       
        return;
        
    }
    
    if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(LEFT) && PS3NavFoot->getButtonPress(CIRCLE)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(LEFT) && PS3NavDome->getButtonPress(CIRCLE))) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnLeft_CIRCLE_type, btnLeft_CIRCLE_MD_func, btnLeft_CIRCLE_cust_MP3_num, btnLeft_CIRCLE_cust_LD_type, btnLeft_CIRCLE_cust_LD_text, btnLeft_CIRCLE_cust_panel, 
                         btnLeft_CIRCLE_use_DP1,
                         btnLeft_CIRCLE_DP1_open_start_delay, 
                         btnLeft_CIRCLE_DP1_stay_open_time,
                         btnLeft_CIRCLE_use_DP2,
                         btnLeft_CIRCLE_DP2_open_start_delay, 
                         btnLeft_CIRCLE_DP2_stay_open_time,
                         btnLeft_CIRCLE_use_DP3,
                         btnLeft_CIRCLE_DP3_open_start_delay, 
                         btnLeft_CIRCLE_DP3_stay_open_time,
                         btnLeft_CIRCLE_use_DP4,
                         btnLeft_CIRCLE_DP4_open_start_delay, 
                         btnLeft_CIRCLE_DP4_stay_open_time,
                         btnLeft_CIRCLE_use_DP5,
                         btnLeft_CIRCLE_DP5_open_start_delay, 
                         btnLeft_CIRCLE_DP5_stay_open_time,
                         btnLeft_CIRCLE_use_DP6,
                         btnLeft_CIRCLE_DP6_open_start_delay, 
                         btnLeft_CIRCLE_DP6_stay_open_time,
                         btnLeft_CIRCLE_use_DP7,
                         btnLeft_CIRCLE_DP7_open_start_delay, 
                         btnLeft_CIRCLE_DP7_stay_open_time,
                         btnLeft_CIRCLE_use_DP8,
                         btnLeft_CIRCLE_DP8_open_start_delay, 
                         btnLeft_CIRCLE_DP8_stay_open_time,
                         btnLeft_CIRCLE_use_DP9,
                         btnLeft_CIRCLE_DP9_open_start_delay, 
                         btnLeft_CIRCLE_DP9_stay_open_time,
                         btnLeft_CIRCLE_use_DP10,
                         btnLeft_CIRCLE_DP10_open_start_delay, 
                         btnLeft_CIRCLE_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnLeft_CIRCLE";
        #endif
      
       
        return;
        
    }

    if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(RIGHT) && PS3NavFoot->getButtonPress(CIRCLE)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(RIGHT) && PS3NavDome->getButtonPress(CIRCLE))) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnRight_CIRCLE_type, btnRight_CIRCLE_MD_func, btnRight_CIRCLE_cust_MP3_num, btnRight_CIRCLE_cust_LD_type, btnRight_CIRCLE_cust_LD_text, btnRight_CIRCLE_cust_panel, 
                         btnRight_CIRCLE_use_DP1,
                         btnRight_CIRCLE_DP1_open_start_delay, 
                         btnRight_CIRCLE_DP1_stay_open_time,
                         btnRight_CIRCLE_use_DP2,
                         btnRight_CIRCLE_DP2_open_start_delay, 
                         btnRight_CIRCLE_DP2_stay_open_time,
                         btnRight_CIRCLE_use_DP3,
                         btnRight_CIRCLE_DP3_open_start_delay, 
                         btnRight_CIRCLE_DP3_stay_open_time,
                         btnRight_CIRCLE_use_DP4,
                         btnRight_CIRCLE_DP4_open_start_delay, 
                         btnRight_CIRCLE_DP4_stay_open_time,
                         btnRight_CIRCLE_use_DP5,
                         btnRight_CIRCLE_DP5_open_start_delay, 
                         btnRight_CIRCLE_DP5_stay_open_time,
                         btnRight_CIRCLE_use_DP6,
                         btnRight_CIRCLE_DP6_open_start_delay, 
                         btnRight_CIRCLE_DP6_stay_open_time,
                         btnRight_CIRCLE_use_DP7,
                         btnRight_CIRCLE_DP7_open_start_delay, 
                         btnRight_CIRCLE_DP7_stay_open_time,
                         btnRight_CIRCLE_use_DP8,
                         btnRight_CIRCLE_DP8_open_start_delay, 
                         btnRight_CIRCLE_DP8_stay_open_time,
                         btnRight_CIRCLE_use_DP9,
                         btnRight_CIRCLE_DP9_open_start_delay, 
                         btnRight_CIRCLE_DP9_stay_open_time,
                         btnRight_CIRCLE_use_DP10,
                         btnRight_CIRCLE_DP10_open_start_delay, 
                         btnRight_CIRCLE_DP10_stay_open_time);
            
        
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnRight_CIRCLE";
        #endif
      
       
        return;
        
    }
    
    //------------------------------------ 
    // Send triggers for the L1 + base buttons 
    //------------------------------------
    if (PS3NavFoot->getButtonPress(UP) && PS3NavFoot->getButtonPress(L1) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnUP_L1_type, btnUP_L1_MD_func, btnUP_L1_cust_MP3_num, btnUP_L1_cust_LD_type, btnUP_L1_cust_LD_text, btnUP_L1_cust_panel, 
                         btnUP_L1_use_DP1,
                         btnUP_L1_DP1_open_start_delay, 
                         btnUP_L1_DP1_stay_open_time,
                         btnUP_L1_use_DP2,
                         btnUP_L1_DP2_open_start_delay, 
                         btnUP_L1_DP2_stay_open_time,
                         btnUP_L1_use_DP3,
                         btnUP_L1_DP3_open_start_delay, 
                         btnUP_L1_DP3_stay_open_time,
                         btnUP_L1_use_DP4,
                         btnUP_L1_DP4_open_start_delay, 
                         btnUP_L1_DP4_stay_open_time,
                         btnUP_L1_use_DP5,
                         btnUP_L1_DP5_open_start_delay, 
                         btnUP_L1_DP5_stay_open_time,
                         btnUP_L1_use_DP6,
                         btnUP_L1_DP6_open_start_delay, 
                         btnUP_L1_DP6_stay_open_time,
                         btnUP_L1_use_DP7,
                         btnUP_L1_DP7_open_start_delay, 
                         btnUP_L1_DP7_stay_open_time,
                         btnUP_L1_use_DP8,
                         btnUP_L1_DP8_open_start_delay, 
                         btnUP_L1_DP8_stay_open_time,
                         btnUP_L1_use_DP9,
                         btnUP_L1_DP9_open_start_delay, 
                         btnUP_L1_DP9_stay_open_time,
                         btnUP_L1_use_DP10,
                         btnUP_L1_DP10_open_start_delay, 
                         btnUP_L1_DP10_stay_open_time);
            
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnUP_L1";
        #endif
      
       
        return;
        
    }
    
    if (PS3NavFoot->getButtonPress(DOWN) && PS3NavFoot->getButtonPress(L1) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnDown_L1_type, btnDown_L1_MD_func, btnDown_L1_cust_MP3_num, btnDown_L1_cust_LD_type, btnDown_L1_cust_LD_text, btnDown_L1_cust_panel, 
                         btnDown_L1_use_DP1,
                         btnDown_L1_DP1_open_start_delay, 
                         btnDown_L1_DP1_stay_open_time,
                         btnDown_L1_use_DP2,
                         btnDown_L1_DP2_open_start_delay, 
                         btnDown_L1_DP2_stay_open_time,
                         btnDown_L1_use_DP3,
                         btnDown_L1_DP3_open_start_delay, 
                         btnDown_L1_DP3_stay_open_time,
                         btnDown_L1_use_DP4,
                         btnDown_L1_DP4_open_start_delay, 
                         btnDown_L1_DP4_stay_open_time,
                         btnDown_L1_use_DP5,
                         btnDown_L1_DP5_open_start_delay, 
                         btnDown_L1_DP5_stay_open_time,
                         btnDown_L1_use_DP6,
                         btnDown_L1_DP6_open_start_delay, 
                         btnDown_L1_DP6_stay_open_time,
                         btnDown_L1_use_DP7,
                         btnDown_L1_DP7_open_start_delay, 
                         btnDown_L1_DP7_stay_open_time,
                         btnDown_L1_use_DP8,
                         btnDown_L1_DP8_open_start_delay, 
                         btnDown_L1_DP8_stay_open_time,
                         btnDown_L1_use_DP9,
                         btnDown_L1_DP9_open_start_delay, 
                         btnDown_L1_DP9_stay_open_time,
                         btnDown_L1_use_DP10,
                         btnDown_L1_DP10_open_start_delay, 
                         btnDown_L1_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnDown_L1";
        #endif
      
       
        return;
        
    }
    
    if (PS3NavFoot->getButtonPress(LEFT) && PS3NavFoot->getButtonPress(L1) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnLeft_L1_type, btnLeft_L1_MD_func, btnLeft_L1_cust_MP3_num, btnLeft_L1_cust_LD_type, btnLeft_L1_cust_LD_text, btnLeft_L1_cust_panel, 
                         btnLeft_L1_use_DP1,
                         btnLeft_L1_DP1_open_start_delay, 
                         btnLeft_L1_DP1_stay_open_time,
                         btnLeft_L1_use_DP2,
                         btnLeft_L1_DP2_open_start_delay, 
                         btnLeft_L1_DP2_stay_open_time,
                         btnLeft_L1_use_DP3,
                         btnLeft_L1_DP3_open_start_delay, 
                         btnLeft_L1_DP3_stay_open_time,
                         btnLeft_L1_use_DP4,
                         btnLeft_L1_DP4_open_start_delay, 
                         btnLeft_L1_DP4_stay_open_time,
                         btnLeft_L1_use_DP5,
                         btnLeft_L1_DP5_open_start_delay, 
                         btnLeft_L1_DP5_stay_open_time,
                         btnLeft_L1_use_DP6,
                         btnLeft_L1_DP6_open_start_delay, 
                         btnLeft_L1_DP6_stay_open_time,
                         btnLeft_L1_use_DP7,
                         btnLeft_L1_DP7_open_start_delay, 
                         btnLeft_L1_DP7_stay_open_time,
                         btnLeft_L1_use_DP8,
                         btnLeft_L1_DP8_open_start_delay, 
                         btnLeft_L1_DP8_stay_open_time,
                         btnLeft_L1_use_DP9,
                         btnLeft_L1_DP9_open_start_delay, 
                         btnLeft_L1_DP9_stay_open_time,
                         btnLeft_L1_use_DP10,
                         btnLeft_L1_DP10_open_start_delay, 
                         btnLeft_L1_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnLeft_L1";
        #endif
      
       
        return;
        
    }

    if (PS3NavFoot->getButtonPress(RIGHT) && PS3NavFoot->getButtonPress(L1) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnRight_L1_type, btnRight_L1_MD_func, btnRight_L1_cust_MP3_num, btnRight_L1_cust_LD_type, btnRight_L1_cust_LD_text, btnRight_L1_cust_panel, 
                         btnRight_L1_use_DP1,
                         btnRight_L1_DP1_open_start_delay, 
                         btnRight_L1_DP1_stay_open_time,
                         btnRight_L1_use_DP2,
                         btnRight_L1_DP2_open_start_delay, 
                         btnRight_L1_DP2_stay_open_time,
                         btnRight_L1_use_DP3,
                         btnRight_L1_DP3_open_start_delay, 
                         btnRight_L1_DP3_stay_open_time,
                         btnRight_L1_use_DP4,
                         btnRight_L1_DP4_open_start_delay, 
                         btnRight_L1_DP4_stay_open_time,
                         btnRight_L1_use_DP5,
                         btnRight_L1_DP5_open_start_delay, 
                         btnRight_L1_DP5_stay_open_time,
                         btnRight_L1_use_DP6,
                         btnRight_L1_DP6_open_start_delay, 
                         btnRight_L1_DP6_stay_open_time,
                         btnRight_L1_use_DP7,
                         btnRight_L1_DP7_open_start_delay, 
                         btnRight_L1_DP7_stay_open_time,
                         btnRight_L1_use_DP8,
                         btnRight_L1_DP8_open_start_delay, 
                         btnRight_L1_DP8_stay_open_time,
                         btnRight_L1_use_DP9,
                         btnRight_L1_DP9_open_start_delay, 
                         btnRight_L1_DP9_stay_open_time,
                         btnRight_L1_use_DP10,
                         btnRight_L1_DP10_open_start_delay, 
                         btnRight_L1_DP10_stay_open_time);
                   
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnRight_L1";
        #endif
      
       
        return;
        
    }
    
    //------------------------------------ 
    // Send triggers for the PS + base buttons 
    //------------------------------------
    if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(UP) && PS3NavFoot->getButtonPress(PS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(UP) && PS3NavDome->getButtonPress(PS))) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnUP_PS_type, btnUP_PS_MD_func, btnUP_PS_cust_MP3_num, btnUP_PS_cust_LD_type, btnUP_PS_cust_LD_text, btnUP_PS_cust_panel, 
                         btnUP_PS_use_DP1,
                         btnUP_PS_DP1_open_start_delay, 
                         btnUP_PS_DP1_stay_open_time,
                         btnUP_PS_use_DP2,
                         btnUP_PS_DP2_open_start_delay, 
                         btnUP_PS_DP2_stay_open_time,
                         btnUP_PS_use_DP3,
                         btnUP_PS_DP3_open_start_delay, 
                         btnUP_PS_DP3_stay_open_time,
                         btnUP_PS_use_DP4,
                         btnUP_PS_DP4_open_start_delay, 
                         btnUP_PS_DP4_stay_open_time,
                         btnUP_PS_use_DP5,
                         btnUP_PS_DP5_open_start_delay, 
                         btnUP_PS_DP5_stay_open_time,
                         btnUP_PS_use_DP6,
                         btnUP_PS_DP6_open_start_delay, 
                         btnUP_PS_DP6_stay_open_time,
                         btnUP_PS_use_DP7,
                         btnUP_PS_DP7_open_start_delay, 
                         btnUP_PS_DP7_stay_open_time,
                         btnUP_PS_use_DP8,
                         btnUP_PS_DP8_open_start_delay, 
                         btnUP_PS_DP8_stay_open_time,
                         btnUP_PS_use_DP9,
                         btnUP_PS_DP9_open_start_delay, 
                         btnUP_PS_DP9_stay_open_time,
                         btnUP_PS_use_DP10,
                         btnUP_PS_DP10_open_start_delay, 
                         btnUP_PS_DP10_stay_open_time);
            
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnUP_PS";
        #endif
      
       
        return;
        
    }
    
    if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(DOWN) && PS3NavFoot->getButtonPress(PS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(DOWN) && PS3NavDome->getButtonPress(PS))) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnDown_PS_type, btnDown_PS_MD_func, btnDown_PS_cust_MP3_num, btnDown_PS_cust_LD_type, btnDown_PS_cust_LD_text, btnDown_PS_cust_panel, 
                         btnDown_PS_use_DP1,
                         btnDown_PS_DP1_open_start_delay, 
                         btnDown_PS_DP1_stay_open_time,
                         btnDown_PS_use_DP2,
                         btnDown_PS_DP2_open_start_delay, 
                         btnDown_PS_DP2_stay_open_time,
                         btnDown_PS_use_DP3,
                         btnDown_PS_DP3_open_start_delay, 
                         btnDown_PS_DP3_stay_open_time,
                         btnDown_PS_use_DP4,
                         btnDown_PS_DP4_open_start_delay, 
                         btnDown_PS_DP4_stay_open_time,
                         btnDown_PS_use_DP5,
                         btnDown_PS_DP5_open_start_delay, 
                         btnDown_PS_DP5_stay_open_time,
                         btnDown_PS_use_DP6,
                         btnDown_PS_DP6_open_start_delay, 
                         btnDown_PS_DP6_stay_open_time,
                         btnDown_PS_use_DP7,
                         btnDown_PS_DP7_open_start_delay, 
                         btnDown_PS_DP7_stay_open_time,
                         btnDown_PS_use_DP8,
                         btnDown_PS_DP8_open_start_delay, 
                         btnDown_PS_DP8_stay_open_time,
                         btnDown_PS_use_DP9,
                         btnDown_PS_DP9_open_start_delay, 
                         btnDown_PS_DP9_stay_open_time,
                         btnDown_PS_use_DP10,
                         btnDown_PS_DP10_open_start_delay, 
                         btnDown_PS_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnDown_PS";
        #endif
      
       
        return;
        
    }
    
    if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(LEFT) && PS3NavFoot->getButtonPress(PS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(LEFT) && PS3NavDome->getButtonPress(PS))) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnLeft_PS_type, btnLeft_PS_MD_func, btnLeft_PS_cust_MP3_num, btnLeft_PS_cust_LD_type, btnLeft_PS_cust_LD_text, btnLeft_PS_cust_panel, 
                         btnLeft_PS_use_DP1,
                         btnLeft_PS_DP1_open_start_delay, 
                         btnLeft_PS_DP1_stay_open_time,
                         btnLeft_PS_use_DP2,
                         btnLeft_PS_DP2_open_start_delay, 
                         btnLeft_PS_DP2_stay_open_time,
                         btnLeft_PS_use_DP3,
                         btnLeft_PS_DP3_open_start_delay, 
                         btnLeft_PS_DP3_stay_open_time,
                         btnLeft_PS_use_DP4,
                         btnLeft_PS_DP4_open_start_delay, 
                         btnLeft_PS_DP4_stay_open_time,
                         btnLeft_PS_use_DP5,
                         btnLeft_PS_DP5_open_start_delay, 
                         btnLeft_PS_DP5_stay_open_time,
                         btnLeft_PS_use_DP6,
                         btnLeft_PS_DP6_open_start_delay, 
                         btnLeft_PS_DP6_stay_open_time,
                         btnLeft_PS_use_DP7,
                         btnLeft_PS_DP7_open_start_delay, 
                         btnLeft_PS_DP7_stay_open_time,
                         btnLeft_PS_use_DP8,
                         btnLeft_PS_DP8_open_start_delay, 
                         btnLeft_PS_DP8_stay_open_time,
                         btnLeft_PS_use_DP9,
                         btnLeft_PS_DP9_open_start_delay, 
                         btnLeft_PS_DP9_stay_open_time,
                         btnLeft_PS_use_DP10,
                         btnLeft_PS_DP10_open_start_delay, 
                         btnLeft_PS_DP10_stay_open_time);
            
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnLeft_PS";
        #endif
      
       
        return;
        
    }

    if (((!PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(RIGHT) && PS3NavFoot->getButtonPress(PS)) || (PS3NavDome->PS3NavigationConnected && PS3NavFoot->getButtonPress(RIGHT) && PS3NavDome->getButtonPress(PS))) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(btnRight_PS_type, btnRight_PS_MD_func, btnRight_PS_cust_MP3_num, btnRight_PS_cust_LD_type, btnRight_PS_cust_LD_text, btnRight_PS_cust_panel, 
                         btnRight_PS_use_DP1,
                         btnRight_PS_DP1_open_start_delay, 
                         btnRight_PS_DP1_stay_open_time,
                         btnRight_PS_use_DP2,
                         btnRight_PS_DP2_open_start_delay, 
                         btnRight_PS_DP2_stay_open_time,
                         btnRight_PS_use_DP3,
                         btnRight_PS_DP3_open_start_delay, 
                         btnRight_PS_DP3_stay_open_time,
                         btnRight_PS_use_DP4,
                         btnRight_PS_DP4_open_start_delay, 
                         btnRight_PS_DP4_stay_open_time,
                         btnRight_PS_use_DP5,
                         btnRight_PS_DP5_open_start_delay, 
                         btnRight_PS_DP5_stay_open_time,
                         btnRight_PS_use_DP6,
                         btnRight_PS_DP6_open_start_delay, 
                         btnRight_PS_DP6_stay_open_time,
                         btnRight_PS_use_DP7,
                         btnRight_PS_DP7_open_start_delay, 
                         btnRight_PS_DP7_stay_open_time,
                         btnRight_PS_use_DP8,
                         btnRight_PS_DP8_open_start_delay, 
                         btnRight_PS_DP8_stay_open_time,
                         btnRight_PS_use_DP9,
                         btnRight_PS_DP9_open_start_delay, 
                         btnRight_PS_DP9_stay_open_time,
                         btnRight_PS_use_DP10,
                         btnRight_PS_DP10_open_start_delay, 
                         btnRight_PS_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "FOOT: btnRight_PS";
        #endif
      
       
        return;
        
    }

}

// ===================================================================================================================
// This function determines if MarcDuino buttons were selected and calls main processing function for DOME Controller
// ===================================================================================================================
void marcDuinoDome()
{
   if (PS3NavDome->PS3NavigationConnected && (PS3NavDome->getButtonPress(UP) || PS3NavDome->getButtonPress(DOWN) || PS3NavDome->getButtonPress(LEFT) || PS3NavDome->getButtonPress(RIGHT)))
   {
      
       if ((millis() - previousMarcDuinoMillis) > 1000)
       {
            marcDuinoButtonCounter = 0;
            previousMarcDuinoMillis = millis();
       } 
     
       marcDuinoButtonCounter += 1;
         
   } else
   {
       return;    
   }
   
   // Clear inbound buffer of any data sent form the MarcDuino board
   while (Serial1.available()) Serial1.read();

    //------------------------------------ 
    // Send triggers for the base buttons 
    //------------------------------------
    if (PS3NavDome->getButtonPress(UP) && !PS3NavDome->getButtonPress(CROSS) && !PS3NavDome->getButtonPress(CIRCLE) && !PS3NavDome->getButtonPress(L1) && !PS3NavDome->getButtonPress(PS) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnUP_MD_func, btnUP_cust_MP3_num, btnUP_cust_LD_type, btnUP_cust_LD_text, btnUP_cust_panel, 
                         btnUP_use_DP1,
                         btnUP_DP1_open_start_delay, 
                         btnUP_DP1_stay_open_time,
                         btnUP_use_DP2,
                         btnUP_DP2_open_start_delay, 
                         btnUP_DP2_stay_open_time,
                         btnUP_use_DP3,
                         btnUP_DP3_open_start_delay, 
                         btnUP_DP3_stay_open_time,
                         btnUP_use_DP4,
                         btnUP_DP4_open_start_delay, 
                         btnUP_DP4_stay_open_time,
                         btnUP_use_DP5,
                         btnUP_DP5_open_start_delay, 
                         btnUP_DP5_stay_open_time,
                         btnUP_use_DP6,
                         btnUP_DP6_open_start_delay, 
                         btnUP_DP6_stay_open_time,
                         btnUP_use_DP7,
                         btnUP_DP7_open_start_delay, 
                         btnUP_DP7_stay_open_time,
                         btnUP_use_DP8,
                         btnUP_DP8_open_start_delay, 
                         btnUP_DP8_stay_open_time,
                         btnUP_use_DP9,
                         btnUP_DP9_open_start_delay, 
                         btnUP_DP9_stay_open_time,
                         btnUP_use_DP10,
                         btnUP_DP10_open_start_delay, 
                         btnUP_DP10_stay_open_time);
            
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnUP";
        #endif
      
        return;
        
    }
    
    if (PS3NavDome->getButtonPress(DOWN) && !PS3NavDome->getButtonPress(CROSS) && !PS3NavDome->getButtonPress(CIRCLE) && !PS3NavDome->getButtonPress(L1) && !PS3NavDome->getButtonPress(PS) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnDown_MD_func, btnDown_cust_MP3_num, btnDown_cust_LD_type, btnDown_cust_LD_text, btnDown_cust_panel, 
                         btnDown_use_DP1,
                         btnDown_DP1_open_start_delay, 
                         btnDown_DP1_stay_open_time,
                         btnDown_use_DP2,
                         btnDown_DP2_open_start_delay, 
                         btnDown_DP2_stay_open_time,
                         btnDown_use_DP3,
                         btnDown_DP3_open_start_delay, 
                         btnDown_DP3_stay_open_time,
                         btnDown_use_DP4,
                         btnDown_DP4_open_start_delay, 
                         btnDown_DP4_stay_open_time,
                         btnDown_use_DP5,
                         btnDown_DP5_open_start_delay, 
                         btnDown_DP5_stay_open_time,
                         btnDown_use_DP6,
                         btnDown_DP6_open_start_delay, 
                         btnDown_DP6_stay_open_time,
                         btnDown_use_DP7,
                         btnDown_DP7_open_start_delay, 
                         btnDown_DP7_stay_open_time,
                         btnDown_use_DP8,
                         btnDown_DP8_open_start_delay, 
                         btnDown_DP8_stay_open_time,
                         btnDown_use_DP9,
                         btnDown_DP9_open_start_delay, 
                         btnDown_DP9_stay_open_time,
                         btnDown_use_DP10,
                         btnDown_DP10_open_start_delay, 
                         btnDown_DP10_stay_open_time);
                         
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnDown";
        #endif
      
        return;      
    }
    
    if (PS3NavDome->getButtonPress(LEFT) && !PS3NavDome->getButtonPress(CROSS) && !PS3NavDome->getButtonPress(CIRCLE) && !PS3NavDome->getButtonPress(L1) && !PS3NavDome->getButtonPress(PS) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnLeft_MD_func, btnLeft_cust_MP3_num, btnLeft_cust_LD_type, btnLeft_cust_LD_text, btnLeft_cust_panel, 
                         btnLeft_use_DP1,
                         btnLeft_DP1_open_start_delay, 
                         btnLeft_DP1_stay_open_time,
                         btnLeft_use_DP2,
                         btnLeft_DP2_open_start_delay, 
                         btnLeft_DP2_stay_open_time,
                         btnLeft_use_DP3,
                         btnLeft_DP3_open_start_delay, 
                         btnLeft_DP3_stay_open_time,
                         btnLeft_use_DP4,
                         btnLeft_DP4_open_start_delay, 
                         btnLeft_DP4_stay_open_time,
                         btnLeft_use_DP5,
                         btnLeft_DP5_open_start_delay, 
                         btnLeft_DP5_stay_open_time,
                         btnLeft_use_DP6,
                         btnLeft_DP6_open_start_delay, 
                         btnLeft_DP6_stay_open_time,
                         btnLeft_use_DP7,
                         btnLeft_DP7_open_start_delay, 
                         btnLeft_DP7_stay_open_time,
                         btnLeft_use_DP8,
                         btnLeft_DP8_open_start_delay, 
                         btnLeft_DP8_stay_open_time,
                         btnLeft_use_DP9,
                         btnLeft_DP9_open_start_delay, 
                         btnLeft_DP9_stay_open_time,
                         btnLeft_use_DP10,
                         btnLeft_DP10_open_start_delay, 
                         btnLeft_DP10_stay_open_time);
                         
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnLeft";
        #endif
       
        return;
        
    }

    if (PS3NavDome->getButtonPress(RIGHT) && !PS3NavDome->getButtonPress(CROSS) && !PS3NavDome->getButtonPress(CIRCLE) && !PS3NavDome->getButtonPress(L1) && !PS3NavDome->getButtonPress(PS) && !PS3NavFoot->getButtonPress(CROSS) && !PS3NavFoot->getButtonPress(CIRCLE) && !PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnRight_MD_func, btnRight_cust_MP3_num, btnRight_cust_LD_type, btnRight_cust_LD_text, btnRight_cust_panel, 
                         btnRight_use_DP1,
                         btnRight_DP1_open_start_delay, 
                         btnRight_DP1_stay_open_time,
                         btnRight_use_DP2,
                         btnRight_DP2_open_start_delay, 
                         btnRight_DP2_stay_open_time,
                         btnRight_use_DP3,
                         btnRight_DP3_open_start_delay, 
                         btnRight_DP3_stay_open_time,
                         btnRight_use_DP4,
                         btnRight_DP4_open_start_delay, 
                         btnRight_DP4_stay_open_time,
                         btnRight_use_DP5,
                         btnRight_DP5_open_start_delay, 
                         btnRight_DP5_stay_open_time,
                         btnRight_use_DP6,
                         btnRight_DP6_open_start_delay, 
                         btnRight_DP6_stay_open_time,
                         btnRight_use_DP7,
                         btnRight_DP7_open_start_delay, 
                         btnRight_DP7_stay_open_time,
                         btnRight_use_DP8,
                         btnRight_DP8_open_start_delay, 
                         btnRight_DP8_stay_open_time,
                         btnRight_use_DP9,
                         btnRight_DP9_open_start_delay, 
                         btnRight_DP9_stay_open_time,
                         btnRight_use_DP10,
                         btnRight_DP10_open_start_delay, 
                         btnRight_DP10_stay_open_time);
                         
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnRight";
        #endif
      
      
        return;
        
    }
    
    //------------------------------------ 
    // Send triggers for the CROSS + base buttons 
    //------------------------------------
    if (PS3NavDome->getButtonPress(UP) && PS3NavFoot->getButtonPress(CROSS) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnUP_CROSS_MD_func, btnUP_CROSS_cust_MP3_num, btnUP_CROSS_cust_LD_type, btnUP_CROSS_cust_LD_text, btnUP_CROSS_cust_panel, 
                         btnUP_CROSS_use_DP1,
                         btnUP_CROSS_DP1_open_start_delay, 
                         btnUP_CROSS_DP1_stay_open_time,
                         btnUP_CROSS_use_DP2,
                         btnUP_CROSS_DP2_open_start_delay, 
                         btnUP_CROSS_DP2_stay_open_time,
                         btnUP_CROSS_use_DP3,
                         btnUP_CROSS_DP3_open_start_delay, 
                         btnUP_CROSS_DP3_stay_open_time,
                         btnUP_CROSS_use_DP4,
                         btnUP_CROSS_DP4_open_start_delay, 
                         btnUP_CROSS_DP4_stay_open_time,
                         btnUP_CROSS_use_DP5,
                         btnUP_CROSS_DP5_open_start_delay, 
                         btnUP_CROSS_DP5_stay_open_time,
                         btnUP_CROSS_use_DP6,
                         btnUP_CROSS_DP6_open_start_delay, 
                         btnUP_CROSS_DP6_stay_open_time,
                         btnUP_CROSS_use_DP7,
                         btnUP_CROSS_DP7_open_start_delay, 
                         btnUP_CROSS_DP7_stay_open_time,
                         btnUP_CROSS_use_DP8,
                         btnUP_CROSS_DP8_open_start_delay, 
                         btnUP_CROSS_DP8_stay_open_time,
                         btnUP_CROSS_use_DP9,
                         btnUP_CROSS_DP9_open_start_delay, 
                         btnUP_CROSS_DP9_stay_open_time,
                         btnUP_CROSS_use_DP10,
                         btnUP_CROSS_DP10_open_start_delay, 
                         btnUP_CROSS_DP10_stay_open_time);
      
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnUP_CROSS";
        #endif
      
      
        return;
        
    }
    
    if (PS3NavDome->getButtonPress(DOWN) && PS3NavFoot->getButtonPress(CROSS) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnDown_CROSS_MD_func, btnDown_CROSS_cust_MP3_num, btnDown_CROSS_cust_LD_type, btnDown_CROSS_cust_LD_text, btnDown_CROSS_cust_panel, 
                         btnDown_CROSS_use_DP1,
                         btnDown_CROSS_DP1_open_start_delay, 
                         btnDown_CROSS_DP1_stay_open_time,
                         btnDown_CROSS_use_DP2,
                         btnDown_CROSS_DP2_open_start_delay, 
                         btnDown_CROSS_DP2_stay_open_time,
                         btnDown_CROSS_use_DP3,
                         btnDown_CROSS_DP3_open_start_delay, 
                         btnDown_CROSS_DP3_stay_open_time,
                         btnDown_CROSS_use_DP4,
                         btnDown_CROSS_DP4_open_start_delay, 
                         btnDown_CROSS_DP4_stay_open_time,
                         btnDown_CROSS_use_DP5,
                         btnDown_CROSS_DP5_open_start_delay, 
                         btnDown_CROSS_DP5_stay_open_time,
                         btnDown_CROSS_use_DP6,
                         btnDown_CROSS_DP6_open_start_delay, 
                         btnDown_CROSS_DP6_stay_open_time,
                         btnDown_CROSS_use_DP7,
                         btnDown_CROSS_DP7_open_start_delay, 
                         btnDown_CROSS_DP7_stay_open_time,
                         btnDown_CROSS_use_DP8,
                         btnDown_CROSS_DP8_open_start_delay, 
                         btnDown_CROSS_DP8_stay_open_time,
                         btnDown_CROSS_use_DP9,
                         btnDown_CROSS_DP9_open_start_delay, 
                         btnDown_CROSS_DP9_stay_open_time,
                         btnDown_CROSS_use_DP10,
                         btnDown_CROSS_DP10_open_start_delay, 
                         btnDown_CROSS_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnDown_CROSS";
        #endif
      
      
        return;
        
    }
    
    if (PS3NavDome->getButtonPress(LEFT) && PS3NavFoot->getButtonPress(CROSS) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnLeft_CROSS_MD_func, btnLeft_CROSS_cust_MP3_num, btnLeft_CROSS_cust_LD_type, btnLeft_CROSS_cust_LD_text, btnLeft_CROSS_cust_panel, 
                         btnLeft_CROSS_use_DP1,
                         btnLeft_CROSS_DP1_open_start_delay, 
                         btnLeft_CROSS_DP1_stay_open_time,
                         btnLeft_CROSS_use_DP2,
                         btnLeft_CROSS_DP2_open_start_delay, 
                         btnLeft_CROSS_DP2_stay_open_time,
                         btnLeft_CROSS_use_DP3,
                         btnLeft_CROSS_DP3_open_start_delay, 
                         btnLeft_CROSS_DP3_stay_open_time,
                         btnLeft_CROSS_use_DP4,
                         btnLeft_CROSS_DP4_open_start_delay, 
                         btnLeft_CROSS_DP4_stay_open_time,
                         btnLeft_CROSS_use_DP5,
                         btnLeft_CROSS_DP5_open_start_delay, 
                         btnLeft_CROSS_DP5_stay_open_time,
                         btnLeft_CROSS_use_DP6,
                         btnLeft_CROSS_DP6_open_start_delay, 
                         btnLeft_CROSS_DP6_stay_open_time,
                         btnLeft_CROSS_use_DP7,
                         btnLeft_CROSS_DP7_open_start_delay, 
                         btnLeft_CROSS_DP7_stay_open_time,
                         btnLeft_CROSS_use_DP8,
                         btnLeft_CROSS_DP8_open_start_delay, 
                         btnLeft_CROSS_DP8_stay_open_time,
                         btnLeft_CROSS_use_DP9,
                         btnLeft_CROSS_DP9_open_start_delay, 
                         btnLeft_CROSS_DP9_stay_open_time,
                         btnLeft_CROSS_use_DP10,
                         btnLeft_CROSS_DP10_open_start_delay, 
                         btnLeft_CROSS_DP10_stay_open_time);
            
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnLeft_CROSS";
        #endif
      
      
        return;
        
    }

    if (PS3NavDome->getButtonPress(RIGHT) && PS3NavFoot->getButtonPress(CROSS) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnRight_CROSS_MD_func, btnRight_CROSS_cust_MP3_num, btnRight_CROSS_cust_LD_type, btnRight_CROSS_cust_LD_text, btnRight_CROSS_cust_panel, 
                         btnRight_CROSS_use_DP1,
                         btnRight_CROSS_DP1_open_start_delay, 
                         btnRight_CROSS_DP1_stay_open_time,
                         btnRight_CROSS_use_DP2,
                         btnRight_CROSS_DP2_open_start_delay, 
                         btnRight_CROSS_DP2_stay_open_time,
                         btnRight_CROSS_use_DP3,
                         btnRight_CROSS_DP3_open_start_delay, 
                         btnRight_CROSS_DP3_stay_open_time,
                         btnRight_CROSS_use_DP4,
                         btnRight_CROSS_DP4_open_start_delay, 
                         btnRight_CROSS_DP4_stay_open_time,
                         btnRight_CROSS_use_DP5,
                         btnRight_CROSS_DP5_open_start_delay, 
                         btnRight_CROSS_DP5_stay_open_time,
                         btnRight_CROSS_use_DP6,
                         btnRight_CROSS_DP6_open_start_delay, 
                         btnRight_CROSS_DP6_stay_open_time,
                         btnRight_CROSS_use_DP7,
                         btnRight_CROSS_DP7_open_start_delay, 
                         btnRight_CROSS_DP7_stay_open_time,
                         btnRight_CROSS_use_DP8,
                         btnRight_CROSS_DP8_open_start_delay, 
                         btnRight_CROSS_DP8_stay_open_time,
                         btnRight_CROSS_use_DP9,
                         btnRight_CROSS_DP9_open_start_delay, 
                         btnRight_CROSS_DP9_stay_open_time,
                         btnRight_CROSS_use_DP10,
                         btnRight_CROSS_DP10_open_start_delay, 
                         btnRight_CROSS_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnRight_CROSS";
        #endif
      
      
        return;
        
    }

    //------------------------------------ 
    // Send triggers for the CIRCLE + base buttons 
    //------------------------------------
    if (PS3NavDome->getButtonPress(UP) && PS3NavFoot->getButtonPress(CIRCLE) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnUP_CIRCLE_MD_func, btnUP_CIRCLE_cust_MP3_num, btnUP_CIRCLE_cust_LD_type, btnUP_CIRCLE_cust_LD_text, btnUP_CIRCLE_cust_panel, 
                         btnUP_CIRCLE_use_DP1,
                         btnUP_CIRCLE_DP1_open_start_delay, 
                         btnUP_CIRCLE_DP1_stay_open_time,
                         btnUP_CIRCLE_use_DP2,
                         btnUP_CIRCLE_DP2_open_start_delay, 
                         btnUP_CIRCLE_DP2_stay_open_time,
                         btnUP_CIRCLE_use_DP3,
                         btnUP_CIRCLE_DP3_open_start_delay, 
                         btnUP_CIRCLE_DP3_stay_open_time,
                         btnUP_CIRCLE_use_DP4,
                         btnUP_CIRCLE_DP4_open_start_delay, 
                         btnUP_CIRCLE_DP4_stay_open_time,
                         btnUP_CIRCLE_use_DP5,
                         btnUP_CIRCLE_DP5_open_start_delay, 
                         btnUP_CIRCLE_DP5_stay_open_time,
                         btnUP_CIRCLE_use_DP6,
                         btnUP_CIRCLE_DP6_open_start_delay, 
                         btnUP_CIRCLE_DP6_stay_open_time,
                         btnUP_CIRCLE_use_DP7,
                         btnUP_CIRCLE_DP7_open_start_delay, 
                         btnUP_CIRCLE_DP7_stay_open_time,
                         btnUP_CIRCLE_use_DP8,
                         btnUP_CIRCLE_DP8_open_start_delay, 
                         btnUP_CIRCLE_DP8_stay_open_time,
                         btnUP_CIRCLE_use_DP9,
                         btnUP_CIRCLE_DP9_open_start_delay, 
                         btnUP_CIRCLE_DP9_stay_open_time,
                         btnUP_CIRCLE_use_DP10,
                         btnUP_CIRCLE_DP10_open_start_delay, 
                         btnUP_CIRCLE_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnUP_CIRCLE";
        #endif
      
      
        return;
        
    }
    
    if (PS3NavDome->getButtonPress(DOWN) && PS3NavFoot->getButtonPress(CIRCLE) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnDown_CIRCLE_MD_func, btnDown_CIRCLE_cust_MP3_num, btnDown_CIRCLE_cust_LD_type, btnDown_CIRCLE_cust_LD_text, btnDown_CIRCLE_cust_panel, 
                         btnDown_CIRCLE_use_DP1,
                         btnDown_CIRCLE_DP1_open_start_delay, 
                         btnDown_CIRCLE_DP1_stay_open_time,
                         btnDown_CIRCLE_use_DP2,
                         btnDown_CIRCLE_DP2_open_start_delay, 
                         btnDown_CIRCLE_DP2_stay_open_time,
                         btnDown_CIRCLE_use_DP3,
                         btnDown_CIRCLE_DP3_open_start_delay, 
                         btnDown_CIRCLE_DP3_stay_open_time,
                         btnDown_CIRCLE_use_DP4,
                         btnDown_CIRCLE_DP4_open_start_delay, 
                         btnDown_CIRCLE_DP4_stay_open_time,
                         btnDown_CIRCLE_use_DP5,
                         btnDown_CIRCLE_DP5_open_start_delay, 
                         btnDown_CIRCLE_DP5_stay_open_time,
                         btnDown_CIRCLE_use_DP6,
                         btnDown_CIRCLE_DP6_open_start_delay, 
                         btnDown_CIRCLE_DP6_stay_open_time,
                         btnDown_CIRCLE_use_DP7,
                         btnDown_CIRCLE_DP7_open_start_delay, 
                         btnDown_CIRCLE_DP7_stay_open_time,
                         btnDown_CIRCLE_use_DP8,
                         btnDown_CIRCLE_DP8_open_start_delay, 
                         btnDown_CIRCLE_DP8_stay_open_time,
                         btnDown_CIRCLE_use_DP9,
                         btnDown_CIRCLE_DP9_open_start_delay, 
                         btnDown_CIRCLE_DP9_stay_open_time,
                         btnDown_CIRCLE_use_DP10,
                         btnDown_CIRCLE_DP10_open_start_delay, 
                         btnDown_CIRCLE_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnDown_CIRCLE";
        #endif
      
      
        return;
        
    }
    
    if (PS3NavDome->getButtonPress(LEFT) && PS3NavFoot->getButtonPress(CIRCLE) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnLeft_CIRCLE_MD_func, btnLeft_CIRCLE_cust_MP3_num, btnLeft_CIRCLE_cust_LD_type, btnLeft_CIRCLE_cust_LD_text, btnLeft_CIRCLE_cust_panel, 
                         btnLeft_CIRCLE_use_DP1,
                         btnLeft_CIRCLE_DP1_open_start_delay, 
                         btnLeft_CIRCLE_DP1_stay_open_time,
                         btnLeft_CIRCLE_use_DP2,
                         btnLeft_CIRCLE_DP2_open_start_delay, 
                         btnLeft_CIRCLE_DP2_stay_open_time,
                         btnLeft_CIRCLE_use_DP3,
                         btnLeft_CIRCLE_DP3_open_start_delay, 
                         btnLeft_CIRCLE_DP3_stay_open_time,
                         btnLeft_CIRCLE_use_DP4,
                         btnLeft_CIRCLE_DP4_open_start_delay, 
                         btnLeft_CIRCLE_DP4_stay_open_time,
                         btnLeft_CIRCLE_use_DP5,
                         btnLeft_CIRCLE_DP5_open_start_delay, 
                         btnLeft_CIRCLE_DP5_stay_open_time,
                         btnLeft_CIRCLE_use_DP6,
                         btnLeft_CIRCLE_DP6_open_start_delay, 
                         btnLeft_CIRCLE_DP6_stay_open_time,
                         btnLeft_CIRCLE_use_DP7,
                         btnLeft_CIRCLE_DP7_open_start_delay, 
                         btnLeft_CIRCLE_DP7_stay_open_time,
                         btnLeft_CIRCLE_use_DP8,
                         btnLeft_CIRCLE_DP8_open_start_delay, 
                         btnLeft_CIRCLE_DP8_stay_open_time,
                         btnLeft_CIRCLE_use_DP9,
                         btnLeft_CIRCLE_DP9_open_start_delay, 
                         btnLeft_CIRCLE_DP9_stay_open_time,
                         btnLeft_CIRCLE_use_DP10,
                         btnLeft_CIRCLE_DP10_open_start_delay, 
                         btnLeft_CIRCLE_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnLeft_CIRCLE";
        #endif
      
      
        return;
        
    }

    if (PS3NavDome->getButtonPress(RIGHT) && PS3NavFoot->getButtonPress(CIRCLE) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnRight_CIRCLE_MD_func, btnRight_CIRCLE_cust_MP3_num, btnRight_CIRCLE_cust_LD_type, btnRight_CIRCLE_cust_LD_text, btnRight_CIRCLE_cust_panel, 
                         btnRight_CIRCLE_use_DP1,
                         btnRight_CIRCLE_DP1_open_start_delay, 
                         btnRight_CIRCLE_DP1_stay_open_time,
                         btnRight_CIRCLE_use_DP2,
                         btnRight_CIRCLE_DP2_open_start_delay, 
                         btnRight_CIRCLE_DP2_stay_open_time,
                         btnRight_CIRCLE_use_DP3,
                         btnRight_CIRCLE_DP3_open_start_delay, 
                         btnRight_CIRCLE_DP3_stay_open_time,
                         btnRight_CIRCLE_use_DP4,
                         btnRight_CIRCLE_DP4_open_start_delay, 
                         btnRight_CIRCLE_DP4_stay_open_time,
                         btnRight_CIRCLE_use_DP5,
                         btnRight_CIRCLE_DP5_open_start_delay, 
                         btnRight_CIRCLE_DP5_stay_open_time,
                         btnRight_CIRCLE_use_DP6,
                         btnRight_CIRCLE_DP6_open_start_delay, 
                         btnRight_CIRCLE_DP6_stay_open_time,
                         btnRight_CIRCLE_use_DP7,
                         btnRight_CIRCLE_DP7_open_start_delay, 
                         btnRight_CIRCLE_DP7_stay_open_time,
                         btnRight_CIRCLE_use_DP8,
                         btnRight_CIRCLE_DP8_open_start_delay, 
                         btnRight_CIRCLE_DP8_stay_open_time,
                         btnRight_CIRCLE_use_DP9,
                         btnRight_CIRCLE_DP9_open_start_delay, 
                         btnRight_CIRCLE_DP9_stay_open_time,
                         btnRight_CIRCLE_use_DP10,
                         btnRight_CIRCLE_DP10_open_start_delay, 
                         btnRight_CIRCLE_DP10_stay_open_time);
            
        
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnRight_CIRCLE";
        #endif
      
      
        return;
        
    }
    
    //------------------------------------ 
    // Send triggers for the L1 + base buttons 
    //------------------------------------
    if (PS3NavDome->getButtonPress(UP) && PS3NavDome->getButtonPress(L1) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnUP_L1_MD_func, btnUP_L1_cust_MP3_num, btnUP_L1_cust_LD_type, btnUP_L1_cust_LD_text, btnUP_L1_cust_panel, 
                         btnUP_L1_use_DP1,
                         btnUP_L1_DP1_open_start_delay, 
                         btnUP_L1_DP1_stay_open_time,
                         btnUP_L1_use_DP2,
                         btnUP_L1_DP2_open_start_delay, 
                         btnUP_L1_DP2_stay_open_time,
                         btnUP_L1_use_DP3,
                         btnUP_L1_DP3_open_start_delay, 
                         btnUP_L1_DP3_stay_open_time,
                         btnUP_L1_use_DP4,
                         btnUP_L1_DP4_open_start_delay, 
                         btnUP_L1_DP4_stay_open_time,
                         btnUP_L1_use_DP5,
                         btnUP_L1_DP5_open_start_delay, 
                         btnUP_L1_DP5_stay_open_time,
                         btnUP_L1_use_DP6,
                         btnUP_L1_DP6_open_start_delay, 
                         btnUP_L1_DP6_stay_open_time,
                         btnUP_L1_use_DP7,
                         btnUP_L1_DP7_open_start_delay, 
                         btnUP_L1_DP7_stay_open_time,
                         btnUP_L1_use_DP8,
                         btnUP_L1_DP8_open_start_delay, 
                         btnUP_L1_DP8_stay_open_time,
                         btnUP_L1_use_DP9,
                         btnUP_L1_DP9_open_start_delay, 
                         btnUP_L1_DP9_stay_open_time,
                         btnUP_L1_use_DP10,
                         btnUP_L1_DP10_open_start_delay, 
                         btnUP_L1_DP10_stay_open_time);
            
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnUP_L1";
        #endif
      
      
        return;
        
    }
    
    if (PS3NavDome->getButtonPress(DOWN) && PS3NavDome->getButtonPress(L1) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnDown_L1_MD_func, btnDown_L1_cust_MP3_num, btnDown_L1_cust_LD_type, btnDown_L1_cust_LD_text, btnDown_L1_cust_panel, 
                         btnDown_L1_use_DP1,
                         btnDown_L1_DP1_open_start_delay, 
                         btnDown_L1_DP1_stay_open_time,
                         btnDown_L1_use_DP2,
                         btnDown_L1_DP2_open_start_delay, 
                         btnDown_L1_DP2_stay_open_time,
                         btnDown_L1_use_DP3,
                         btnDown_L1_DP3_open_start_delay, 
                         btnDown_L1_DP3_stay_open_time,
                         btnDown_L1_use_DP4,
                         btnDown_L1_DP4_open_start_delay, 
                         btnDown_L1_DP4_stay_open_time,
                         btnDown_L1_use_DP5,
                         btnDown_L1_DP5_open_start_delay, 
                         btnDown_L1_DP5_stay_open_time,
                         btnDown_L1_use_DP6,
                         btnDown_L1_DP6_open_start_delay, 
                         btnDown_L1_DP6_stay_open_time,
                         btnDown_L1_use_DP7,
                         btnDown_L1_DP7_open_start_delay, 
                         btnDown_L1_DP7_stay_open_time,
                         btnDown_L1_use_DP8,
                         btnDown_L1_DP8_open_start_delay, 
                         btnDown_L1_DP8_stay_open_time,
                         btnDown_L1_use_DP9,
                         btnDown_L1_DP9_open_start_delay, 
                         btnDown_L1_DP9_stay_open_time,
                         btnDown_L1_use_DP10,
                         btnDown_L1_DP10_open_start_delay, 
                         btnDown_L1_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnDown_L1";
        #endif
      
      
        return;
        
    }
    
    if (PS3NavDome->getButtonPress(LEFT) && PS3NavDome->getButtonPress(L1) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnLeft_L1_MD_func, btnLeft_L1_cust_MP3_num, btnLeft_L1_cust_LD_type, btnLeft_L1_cust_LD_text, btnLeft_L1_cust_panel, 
                         btnLeft_L1_use_DP1,
                         btnLeft_L1_DP1_open_start_delay, 
                         btnLeft_L1_DP1_stay_open_time,
                         btnLeft_L1_use_DP2,
                         btnLeft_L1_DP2_open_start_delay, 
                         btnLeft_L1_DP2_stay_open_time,
                         btnLeft_L1_use_DP3,
                         btnLeft_L1_DP3_open_start_delay, 
                         btnLeft_L1_DP3_stay_open_time,
                         btnLeft_L1_use_DP4,
                         btnLeft_L1_DP4_open_start_delay, 
                         btnLeft_L1_DP4_stay_open_time,
                         btnLeft_L1_use_DP5,
                         btnLeft_L1_DP5_open_start_delay, 
                         btnLeft_L1_DP5_stay_open_time,
                         btnLeft_L1_use_DP6,
                         btnLeft_L1_DP6_open_start_delay, 
                         btnLeft_L1_DP6_stay_open_time,
                         btnLeft_L1_use_DP7,
                         btnLeft_L1_DP7_open_start_delay, 
                         btnLeft_L1_DP7_stay_open_time,
                         btnLeft_L1_use_DP8,
                         btnLeft_L1_DP8_open_start_delay, 
                         btnLeft_L1_DP8_stay_open_time,
                         btnLeft_L1_use_DP9,
                         btnLeft_L1_DP9_open_start_delay, 
                         btnLeft_L1_DP9_stay_open_time,
                         btnLeft_L1_use_DP10,
                         btnLeft_L1_DP10_open_start_delay, 
                         btnLeft_L1_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnLeft_L1";
        #endif
      
      
        return;
        
    }

    if (PS3NavDome->getButtonPress(RIGHT) && PS3NavDome->getButtonPress(L1) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnRight_L1_MD_func, btnRight_L1_cust_MP3_num, btnRight_L1_cust_LD_type, btnRight_L1_cust_LD_text, btnRight_L1_cust_panel, 
                         btnRight_L1_use_DP1,
                         btnRight_L1_DP1_open_start_delay, 
                         btnRight_L1_DP1_stay_open_time,
                         btnRight_L1_use_DP2,
                         btnRight_L1_DP2_open_start_delay, 
                         btnRight_L1_DP2_stay_open_time,
                         btnRight_L1_use_DP3,
                         btnRight_L1_DP3_open_start_delay, 
                         btnRight_L1_DP3_stay_open_time,
                         btnRight_L1_use_DP4,
                         btnRight_L1_DP4_open_start_delay, 
                         btnRight_L1_DP4_stay_open_time,
                         btnRight_L1_use_DP5,
                         btnRight_L1_DP5_open_start_delay, 
                         btnRight_L1_DP5_stay_open_time,
                         btnRight_L1_use_DP6,
                         btnRight_L1_DP6_open_start_delay, 
                         btnRight_L1_DP6_stay_open_time,
                         btnRight_L1_use_DP7,
                         btnRight_L1_DP7_open_start_delay, 
                         btnRight_L1_DP7_stay_open_time,
                         btnRight_L1_use_DP8,
                         btnRight_L1_DP8_open_start_delay, 
                         btnRight_L1_DP8_stay_open_time,
                         btnRight_L1_use_DP9,
                         btnRight_L1_DP9_open_start_delay, 
                         btnRight_L1_DP9_stay_open_time,
                         btnRight_L1_use_DP10,
                         btnRight_L1_DP10_open_start_delay, 
                         btnRight_L1_DP10_stay_open_time);
                   
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnRight_L1";
        #endif
      
      
        return;
        
    }
    
    //------------------------------------ 
    // Send triggers for the PS + base buttons 
    //------------------------------------
    if (PS3NavDome->getButtonPress(UP) && PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnUP_PS_MD_func, btnUP_PS_cust_MP3_num, btnUP_PS_cust_LD_type, btnUP_PS_cust_LD_text, btnUP_PS_cust_panel, 
                         btnUP_PS_use_DP1,
                         btnUP_PS_DP1_open_start_delay, 
                         btnUP_PS_DP1_stay_open_time,
                         btnUP_PS_use_DP2,
                         btnUP_PS_DP2_open_start_delay, 
                         btnUP_PS_DP2_stay_open_time,
                         btnUP_PS_use_DP3,
                         btnUP_PS_DP3_open_start_delay, 
                         btnUP_PS_DP3_stay_open_time,
                         btnUP_PS_use_DP4,
                         btnUP_PS_DP4_open_start_delay, 
                         btnUP_PS_DP4_stay_open_time,
                         btnUP_PS_use_DP5,
                         btnUP_PS_DP5_open_start_delay, 
                         btnUP_PS_DP5_stay_open_time,
                         btnUP_PS_use_DP6,
                         btnUP_PS_DP6_open_start_delay, 
                         btnUP_PS_DP6_stay_open_time,
                         btnUP_PS_use_DP7,
                         btnUP_PS_DP7_open_start_delay, 
                         btnUP_PS_DP7_stay_open_time,
                         btnUP_PS_use_DP8,
                         btnUP_PS_DP8_open_start_delay, 
                         btnUP_PS_DP8_stay_open_time,
                         btnUP_PS_use_DP9,
                         btnUP_PS_DP9_open_start_delay, 
                         btnUP_PS_DP9_stay_open_time,
                         btnUP_PS_use_DP10,
                         btnUP_PS_DP10_open_start_delay, 
                         btnUP_PS_DP10_stay_open_time);
            
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnUP_PS";
        #endif
       
        return;
        
    }
    
    if (PS3NavDome->getButtonPress(DOWN) && PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnDown_PS_MD_func, btnDown_PS_cust_MP3_num, btnDown_PS_cust_LD_type, btnDown_PS_cust_LD_text, btnDown_PS_cust_panel, 
                         btnDown_PS_use_DP1,
                         btnDown_PS_DP1_open_start_delay, 
                         btnDown_PS_DP1_stay_open_time,
                         btnDown_PS_use_DP2,
                         btnDown_PS_DP2_open_start_delay, 
                         btnDown_PS_DP2_stay_open_time,
                         btnDown_PS_use_DP3,
                         btnDown_PS_DP3_open_start_delay, 
                         btnDown_PS_DP3_stay_open_time,
                         btnDown_PS_use_DP4,
                         btnDown_PS_DP4_open_start_delay, 
                         btnDown_PS_DP4_stay_open_time,
                         btnDown_PS_use_DP5,
                         btnDown_PS_DP5_open_start_delay, 
                         btnDown_PS_DP5_stay_open_time,
                         btnDown_PS_use_DP6,
                         btnDown_PS_DP6_open_start_delay, 
                         btnDown_PS_DP6_stay_open_time,
                         btnDown_PS_use_DP7,
                         btnDown_PS_DP7_open_start_delay, 
                         btnDown_PS_DP7_stay_open_time,
                         btnDown_PS_use_DP8,
                         btnDown_PS_DP8_open_start_delay, 
                         btnDown_PS_DP8_stay_open_time,
                         btnDown_PS_use_DP9,
                         btnDown_PS_DP9_open_start_delay, 
                         btnDown_PS_DP9_stay_open_time,
                         btnDown_PS_use_DP10,
                         btnDown_PS_DP10_open_start_delay, 
                         btnDown_PS_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnDown_PS";
        #endif
      
      
        return;
        
    }
    
    if (PS3NavDome->getButtonPress(LEFT) && PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnLeft_PS_MD_func, btnLeft_PS_cust_MP3_num, btnLeft_PS_cust_LD_type, btnLeft_PS_cust_LD_text, btnLeft_PS_cust_panel, 
                         btnLeft_PS_use_DP1,
                         btnLeft_PS_DP1_open_start_delay, 
                         btnLeft_PS_DP1_stay_open_time,
                         btnLeft_PS_use_DP2,
                         btnLeft_PS_DP2_open_start_delay, 
                         btnLeft_PS_DP2_stay_open_time,
                         btnLeft_PS_use_DP3,
                         btnLeft_PS_DP3_open_start_delay, 
                         btnLeft_PS_DP3_stay_open_time,
                         btnLeft_PS_use_DP4,
                         btnLeft_PS_DP4_open_start_delay, 
                         btnLeft_PS_DP4_stay_open_time,
                         btnLeft_PS_use_DP5,
                         btnLeft_PS_DP5_open_start_delay, 
                         btnLeft_PS_DP5_stay_open_time,
                         btnLeft_PS_use_DP6,
                         btnLeft_PS_DP6_open_start_delay, 
                         btnLeft_PS_DP6_stay_open_time,
                         btnLeft_PS_use_DP7,
                         btnLeft_PS_DP7_open_start_delay, 
                         btnLeft_PS_DP7_stay_open_time,
                         btnLeft_PS_use_DP8,
                         btnLeft_PS_DP8_open_start_delay, 
                         btnLeft_PS_DP8_stay_open_time,
                         btnLeft_PS_use_DP9,
                         btnLeft_PS_DP9_open_start_delay, 
                         btnLeft_PS_DP9_stay_open_time,
                         btnLeft_PS_use_DP10,
                         btnLeft_PS_DP10_open_start_delay, 
                         btnLeft_PS_DP10_stay_open_time);
            
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnLeft_PS";
        #endif
      
      
        return;
        
    }

    if (PS3NavDome->getButtonPress(RIGHT) && PS3NavFoot->getButtonPress(PS) && marcDuinoButtonCounter == 1)
    {
      
       marcDuinoButtonPush(1, FTbtnRight_PS_MD_func, btnRight_PS_cust_MP3_num, btnRight_PS_cust_LD_type, btnRight_PS_cust_LD_text, btnRight_PS_cust_panel, 
                         btnRight_PS_use_DP1,
                         btnRight_PS_DP1_open_start_delay, 
                         btnRight_PS_DP1_stay_open_time,
                         btnRight_PS_use_DP2,
                         btnRight_PS_DP2_open_start_delay, 
                         btnRight_PS_DP2_stay_open_time,
                         btnRight_PS_use_DP3,
                         btnRight_PS_DP3_open_start_delay, 
                         btnRight_PS_DP3_stay_open_time,
                         btnRight_PS_use_DP4,
                         btnRight_PS_DP4_open_start_delay, 
                         btnRight_PS_DP4_stay_open_time,
                         btnRight_PS_use_DP5,
                         btnRight_PS_DP5_open_start_delay, 
                         btnRight_PS_DP5_stay_open_time,
                         btnRight_PS_use_DP6,
                         btnRight_PS_DP6_open_start_delay, 
                         btnRight_PS_DP6_stay_open_time,
                         btnRight_PS_use_DP7,
                         btnRight_PS_DP7_open_start_delay, 
                         btnRight_PS_DP7_stay_open_time,
                         btnRight_PS_use_DP8,
                         btnRight_PS_DP8_open_start_delay, 
                         btnRight_PS_DP8_stay_open_time,
                         btnRight_PS_use_DP9,
                         btnRight_PS_DP9_open_start_delay, 
                         btnRight_PS_DP9_stay_open_time,
                         btnRight_PS_use_DP10,
                         btnRight_PS_DP10_open_start_delay, 
                         btnRight_PS_DP10_stay_open_time);
                    
        #ifdef SHADOW_VERBOSE      
             output += "DOME: btnRight_PS";
        #endif
      
      
        return;
        
    }

}


// =======================================================================================
// This function handles the processing of custom MarcDuino panel routines
// =======================================================================================
void custMarcDuinoPanel()
{
  
      // Open & Close Logic: Dome Panel #1
      if (DP1_Status == 1)
      {
        
         if ((DP1_start + (DP1_s_delay * 1000)) < millis())
         {
           
             Serial1.print(":OP01\r");
             DP1_Status = 2;
         }
        
      }
      
      if (DP1_Status == 2)
      {
        
         if ((DP1_start + ((DP1_s_delay + DP1_o_time) * 1000)) < millis())
         {
           
             Serial1.print(":CL01\r");
             DP1_Status = 0;
         }        
        
      }
      
      // Open & Close Logic: Dome Panel #2
      if (DP2_Status == 1)
      {
        
         if ((DP2_start + (DP2_s_delay * 1000)) < millis())
         {
           
             Serial1.print(":OP02\r");
             DP2_Status = 2;
         }
        
      }
      
      if (DP2_Status == 2)
      {
        
         if ((DP2_start + ((DP2_s_delay + DP2_o_time) * 1000)) < millis())
         {
           
             Serial1.print(":CL02\r");
             DP2_Status = 0;
         }        
        
      } 
 
      // Open & Close Logic: Dome Panel #3
      if (DP3_Status == 1)
      {
        
         if ((DP3_start + (DP3_s_delay * 1000)) < millis())
         {
           
             Serial1.print(":OP03\r");
             DP3_Status = 2;
         }
        
      }
      
      if (DP3_Status == 2)
      {
        
         if ((DP3_start + ((DP3_s_delay + DP3_o_time) * 1000)) < millis())
         {
           
             Serial1.print(":CL03\r");
             DP3_Status = 0;
         }        
        
      } 
      
      // Open & Close Logic: Dome Panel #4
      if (DP4_Status == 1)
      {
        
         if ((DP4_start + (DP4_s_delay * 1000)) < millis())
         {
           
             Serial1.print(":OP04\r");
             DP4_Status = 2;
         }
        
      }
      
      if (DP4_Status == 2)
      {
        
         if ((DP4_start + ((DP4_s_delay + DP4_o_time) * 1000)) < millis())
         {
           
             Serial1.print(":CL04\r");
             DP4_Status = 0;
         }        
        
      }
      
      // Open & Close Logic: Dome Panel #5
      if (DP5_Status == 1)
      {
        
         if ((DP5_start + (DP5_s_delay * 1000)) < millis())
         {
           
             Serial1.print(":OP05\r");
             DP5_Status = 2;
         }
        
      }
      
      if (DP5_Status == 2)
      {
        
         if ((DP5_start + ((DP5_s_delay + DP5_o_time) * 1000)) < millis())
         {
           
             Serial1.print(":CL05\r");
             DP5_Status = 0;
         }        
        
      }
      
      // Open & Close Logic: Dome Panel #6
      if (DP6_Status == 1)
      {
        
         if ((DP6_start + (DP6_s_delay * 1000)) < millis())
         {
           
             Serial1.print(":OP06\r");
             DP6_Status = 2;
         }
        
      }
      
      if (DP6_Status == 2)
      {
        
         if ((DP6_start + ((DP6_s_delay + DP6_o_time) * 1000)) < millis())
         {
           
             Serial1.print(":CL06\r");
             DP6_Status = 0;
         }        
        
      }
      
      // Open & Close Logic: Dome Panel #7
      if (DP7_Status == 1)
      {
        
         if ((DP7_start + (DP7_s_delay * 1000)) < millis())
         {
           
             Serial1.print(":OP07\r");
             DP7_Status = 2;
         }
        
      }
      
      if (DP7_Status == 2)
      {
        
         if ((DP7_start + ((DP7_s_delay + DP7_o_time) * 1000)) < millis())
         {
           
             Serial1.print(":CL07\r");
             DP7_Status = 0;
         }        
        
      }

      // Open & Close Logic: Dome Panel #8
      if (DP8_Status == 1)
      {
        
         if ((DP8_start + (DP8_s_delay * 1000)) < millis())
         {
           
             Serial1.print(":OP08\r");
             DP8_Status = 2;
         }
        
      }
      
      if (DP8_Status == 2)
      {
        
         if ((DP8_start + ((DP8_s_delay + DP8_o_time) * 1000)) < millis())
         {
           
             Serial1.print(":CL08\r");
             DP8_Status = 0;
         }        
        
      }
      
      // Open & Close Logic: Dome Panel #9
      if (DP9_Status == 1)
      {
        
         if ((DP9_start + (DP9_s_delay * 1000)) < millis())
         {
           
             Serial1.print(":OP09\r");
             DP9_Status = 2;
         }
        
      }
      
      if (DP9_Status == 2)
      {
        
         if ((DP9_start + ((DP9_s_delay + DP9_o_time) * 1000)) < millis())
         {
           
             Serial1.print(":CL09\r");
             DP9_Status = 0;
         }        
        
      }
      
      // Open & Close Logic: Dome Panel #10
      if (DP10_Status == 1)
      {
        
         if ((DP10_start + (DP10_s_delay * 1000)) < millis())
         {
           
             Serial1.print(":OP10\r");
             DP10_Status = 2;
         }
        
      }
      
      if (DP10_Status == 2)
      {
        
         if ((DP10_start + ((DP10_s_delay + DP10_o_time) * 1000)) < millis())
         {
           
             Serial1.print(":CL10\r");
             DP10_Status = 0;
         }        
        
      }
      
      // If all the panels have now closed - close out the custom routine
      if (DP1_Status + DP2_Status + DP3_Status + DP4_Status + DP5_Status + DP6_Status + DP7_Status + DP8_Status + DP9_Status + DP10_Status == 0)
      {
        
          runningCustRoutine = false;
        
      }
}

// =======================================================================================
//                             Dome Automation Function
//
//    Features toggles 'on' via L2 + CIRCLE.  'off' via L2 + CROSS.  Default is 'off'.
//
//    This routines randomly turns the dome motor in both directions.  It assumes the 
//    dome is in the 'home' position when the auto dome feature is toggled on.  From
//    there it turns the dome in a random direction.  Stops for a random length of 
//    of time.  Then returns the dome to the home position.  This randomly repeats.
//
//    It is driven off the user variable - time360DomeTurn.  This records how long
//    it takes the dome to do a 360 degree turn at the given auto dome speed.  Tweaking
//    this parameter to be close provides the best results.
//
//    Activating the dome controller manually immediately cancels the auto dome feature
//    or you can toggle the feature off by pressing L2 + CROSS.
// =======================================================================================
void autoDome()
{
    long rndNum;
    int domeSpeed;
    
    if (domeStatus == 0)  // Dome is currently stopped - prepare for a future turn
    {
      
        if (domeTargetPosition == 0)  // Dome is currently in the home position - prepare to turn away
        {
          
            domeStartTurnTime = millis() + (random(3, 10) * 1000);
            
            rndNum = random(5,354);
            
            domeTargetPosition = rndNum;  // set the target position to a random degree of a 360 circle - shaving off the first and last 5 degrees
            
            if (domeTargetPosition < 180)  // Turn the dome in the positive direction
            {
              
                domeTurnDirection = 1;
                
                domeStopTurnTime = domeStartTurnTime + ((domeTargetPosition / 360) * time360DomeTurn);
              
            } else  // Turn the dome in the negative direction
            {
                    
                domeTurnDirection = -1;
                
                domeStopTurnTime = domeStartTurnTime + (((360 - domeTargetPosition) / 360) * time360DomeTurn);
              
            }
          
        } else  // Dome is not in the home position - send it back to home
        {
          
            domeStartTurnTime = millis() + (random(3, 10) * 1000);
            
            if (domeTargetPosition < 180)
            {
              
                domeTurnDirection = -1;
                
                domeStopTurnTime = domeStartTurnTime + ((domeTargetPosition / 360) * time360DomeTurn);
              
            } else
            {
                    
                domeTurnDirection = 1;
                
                domeStopTurnTime = domeStartTurnTime + (((360 - domeTargetPosition) / 360) * time360DomeTurn);
              
            }
            
            domeTargetPosition = 0;
          
        }
      
        domeStatus = 1;  // Set dome status to preparing for a future turn
               
        #ifdef SHADOW_DEBUG
          output += "Dome Automation: Initial Turn Set\r\n";
          output +=  "Current Time: ";
          output +=  millis();
          output += "\r\n Next Start Time: ";
          output += domeStartTurnTime;
          output += "\r\n";
          output += "Next Stop Time: ";
          output += domeStopTurnTime;
          output += "\r\n";          
          output += "Dome Target Position: ";
          output += domeTargetPosition;
          output += "\r\n";          
        #endif

    }
    
    
    if (domeStatus == 1)  // Dome is prepared for a future move - start the turn when ready
    {
      
        if (domeStartTurnTime < millis())
        {
          
             domeStatus = 2; 
             
             #ifdef SHADOW_DEBUG
                output += "Dome Automation: Ready To Start Turn\r\n";
             #endif
          
        }
    }
    
    if (domeStatus == 2) // Dome is now actively turning until it reaches its stop time
    {
      
        if (domeStopTurnTime > millis())
        {
          
              domeSpeed = domeAutoSpeed * domeTurnDirection;
          
              SyR->motor(domeSpeed);

             #ifdef SHADOW_DEBUG
                output += "Turning Now!!\r\n";
             #endif
          
          
        } else  // turn completed - stop the motor
        {
              domeStatus = 0;
              SyR->stop();

              #ifdef SHADOW_DEBUG
                 output += "STOP TURN!!\r\n";
              #endif
        }
      
    }
  
}

// =======================================================================================
//           Program Utility Functions - Called from various locations
// =======================================================================================

// =======================================================================================
//           PPS3 Controller Device Mgt Functions
// =======================================================================================

void onInitPS3NavFoot()
{
    String btAddress = getLastConnectedBtMAC();
    PS3NavFoot->setLedOn(LED1);
    isPS3NavigatonInitialized = true;
    badPS3Data = 0;

    #ifdef SHADOW_DEBUG
      output += "\r\nBT Address of Last connected Device when FOOT PS3 Connected: ";
      output += btAddress;
    #endif
    
    if (btAddress == PS3ControllerFootMac || btAddress == PS3ControllerBackupFootMac)
    {
        
          #ifdef SHADOW_DEBUG
             output += "\r\nWe have our FOOT controller connected. "+String(btAddress)+" \r\n";
          #endif
          
          mainControllerConnected = true;
          WaitingforReconnect = true;
          
    } else
    {
      
        // Prevent connection from anything but the MAIN controllers          
        #ifdef SHADOW_DEBUG
              output += "\r\nWe have an invalid controller ("+String(btAddress)+") trying to connect as tha FOOT controller, it will be dropped.\r\n";
        #endif

        stopFeet();
        SyR->stop();
        isFootMotorStopped = true;
        footDriveSpeed = 0;
        PS3NavFoot->setLedOff(LED1);
        PS3NavFoot->disconnect();
        printOutput();
    
        isPS3NavigatonInitialized = false;
        mainControllerConnected = false;
        
    } 
}

void onInitPS3NavDome()
{
    String btAddress = getLastConnectedBtMAC();
    PS3NavDome->setLedOn(LED1);
    isSecondaryPS3NavigatonInitialized = true;
    badPS3Data = 0;
    
    if (btAddress == PS3ControllerDomeMAC || btAddress == PS3ControllerBackupDomeMAC)
    {
        
          #ifdef SHADOW_DEBUG
             output += "\r\nWe have our DOME controller connected. "+String(btAddress)+" \r\n";
          #endif
          
          domeControllerConnected = true;
          WaitingforReconnectDome = true;
          
    } else
    {
      
        // Prevent connection from anything but the DOME controllers          
        #ifdef SHADOW_DEBUG
              output += "\r\nWe have an invalid controller ("+String(btAddress)+") trying to connect as the DOME controller, it will be dropped.\r\n";
        #endif

        stopFeet();
        SyR->stop();
        isFootMotorStopped = true;
        footDriveSpeed = 0;
        PS3NavDome->setLedOff(LED1);
        PS3NavDome->disconnect();
        printOutput();
    
        isSecondaryPS3NavigatonInitialized = false;
        domeControllerConnected = false;
        
    } 
}

String getLastConnectedBtMAC()
{
    String btAddress = "";
    for(int8_t i = 5; i > 0; i--)
    {
        if (Btd.disc_bdaddr[i]<0x10)
        {
            btAddress +="0";
        }
        btAddress += String(Btd.disc_bdaddr[i], HEX);
        btAddress +=(":");
    }
    btAddress += String(Btd.disc_bdaddr[0], HEX);
    btAddress.toUpperCase();
    return btAddress; 
}

boolean criticalFaultDetect()
{
    if (PS3NavFoot->PS3NavigationConnected || PS3NavFoot->PS3Connected)
    {
        
        currentTime = millis();
        lastMsgTime = PS3NavFoot->getLastMessageTime();
        msgLagTime = currentTime - lastMsgTime;            
        
        if (WaitingforReconnect)
        {
            
            if (msgLagTime < 200)
            {
             
                WaitingforReconnect = false; 
            
            }
            
            lastMsgTime = currentTime;
            
        } 
        
        if ( currentTime >= lastMsgTime)
        {
              msgLagTime = currentTime - lastMsgTime;
              
        } else
        {

             msgLagTime = 0;
        }
        
        if (msgLagTime > 300 && !isFootMotorStopped)
        {
            #ifdef SHADOW_DEBUG
              output += "It has been 300ms since we heard from the PS3 Foot Controller\r\n";
              output += "Shut downing motors, and watching for a new PS3 Foot message\r\n";
            #endif
            stopFeet();
            isFootMotorStopped = true;
            footDriveSpeed = 0;
        }
        
        if ( msgLagTime > 10000 )
        {
            #ifdef SHADOW_DEBUG
              output += "It has been 10s since we heard from the PS3 Foot Controller\r\n";
              output += "msgLagTime:";
              output += msgLagTime;
              output += "  lastMsgTime:";
              output += lastMsgTime;
              output += "  millis:";
              output += millis();            
              output += "\r\nDisconnecting the Foot controller.\r\n";
            #endif
            stopFeet();
            isFootMotorStopped = true;
            footDriveSpeed = 0;
            PS3NavFoot->disconnect();
            WaitingforReconnect = true;
            return true;
        }

        //Check PS3 Signal Data
        if(!PS3NavFoot->getStatus(Plugged) && !PS3NavFoot->getStatus(Unplugged))
        {
            //We don't have good data from the controller.
            //Wait 15ms if no second controller - 100ms if some controller connected, Update USB, and try again
            if (PS3NavDome->PS3NavigationConnected)
            {
                  delay(100);     
            } else
            {
                  delay(15);
            }
            
            Usb.Task();   
            lastMsgTime = PS3NavFoot->getLastMessageTime();
            
            if(!PS3NavFoot->getStatus(Plugged) && !PS3NavFoot->getStatus(Unplugged))
            {
                badPS3Data++;
                #ifdef SHADOW_DEBUG
                    output += "\r\n**Invalid data from PS3 FOOT Controller. - Resetting Data**\r\n";
                #endif
                return true;
            }
        }
        else if (badPS3Data > 0)
        {

            badPS3Data = 0;
        }
        
        if ( badPS3Data > 10 )
        {
            #ifdef SHADOW_DEBUG
                output += "Too much bad data coming from the PS3 FOOT Controller\r\n";
                output += "Disconnecting the controller and stop motors.\r\n";
            #endif
            stopFeet();
            isFootMotorStopped = true;
            footDriveSpeed = 0;
            PS3NavFoot->disconnect();
            WaitingforReconnect = true;
            return true;
        }
    }
    else if (!isFootMotorStopped) //PS controller is not connected, but foot motors aren't stopped. This is madness!
    {
        #ifdef SHADOW_DEBUG      
            output += "No foot controller was found\r\n";
            output += "Shuting down motors and watching for a new PS3 foot message\r\n";
        #endif
        stopFeet();
        isFootMotorStopped = true;
        footDriveSpeed = 0;
        WaitingforReconnect = true;
        return true;
    }
    
    return false;
}

boolean criticalFaultDetectDome()
{
    if (PS3NavDome->PS3NavigationConnected || PS3NavDome->PS3Connected)
    {

        currentTime = millis();
        lastMsgTime = PS3NavDome->getLastMessageTime();
        msgLagTime = currentTime - lastMsgTime;            
        
        if (WaitingforReconnectDome)
        {
            if (msgLagTime < 200)
            {
             
                WaitingforReconnectDome = false; 
            
            }
            
            lastMsgTime = currentTime;
            
        }
        
        if ( currentTime >= lastMsgTime)
        {
             msgLagTime = currentTime - lastMsgTime;
              
        } else
        {
             msgLagTime = 0;
        }
        
        if ( msgLagTime > 10000 )
        {
            #ifdef SHADOW_DEBUG
              output += "It has been 10s since we heard from the PS3 Dome Controller\r\n";
              output += "msgLagTime:";
              output += msgLagTime;
              output += "  lastMsgTime:";
              output += lastMsgTime;
              output += "  millis:";
              output += millis();            
              output += "\r\nDisconnecting the Dome controller.\r\n";
            #endif
            
            SyR->stop();
            PS3NavDome->disconnect();
            WaitingforReconnectDome = true;
            return true;
        }

        //Check PS3 Signal Data
        if(!PS3NavDome->getStatus(Plugged) && !PS3NavDome->getStatus(Unplugged))
        {

            // We don't have good data from the controller.
            //Wait 100ms, Update USB, and try again
            delay(100);
            
            Usb.Task();
            lastMsgTime = PS3NavDome->getLastMessageTime();
            
            if(!PS3NavDome->getStatus(Plugged) && !PS3NavDome->getStatus(Unplugged))
            {
                badPS3DataDome++;
                #ifdef SHADOW_DEBUG
                    output += "\r\n**Invalid data from PS3 Dome Controller. - Resetting Data**\r\n";
                #endif
                return true;
            }
        } else if (badPS3DataDome > 0)
        {
             badPS3DataDome = 0;
        }
        
        if ( badPS3DataDome > 10 )
        {
            #ifdef SHADOW_DEBUG
                output += "Too much bad data coming from the PS3 DOME Controller\r\n";
                output += "Disconnecting the controller and stop motors.\r\n";
            #endif
            SyR->stop();
            PS3NavDome->disconnect();
            WaitingforReconnectDome = true;
            return true;
        }
    } 
    
    return false;
}

// =======================================================================================
//           USB Read Function - Supports Main Program Loop
// =======================================================================================

boolean readUSB()
{
  
     Usb.Task();
     
    //The more devices we have connected to the USB or BlueTooth, the more often Usb.Task need to be called to eliminate latency.
    if (PS3NavFoot->PS3NavigationConnected) 
    {
        if (criticalFaultDetect())
        {
            //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
            printOutput();
            return false;
        }
        
    } else if (!isFootMotorStopped)
    {
        #ifdef SHADOW_DEBUG      
            output += "No foot controller was found\r\n";
            output += "Shuting down motors, and watching for a new PS3 foot message\r\n";
        #endif
        stopFeet();
        isFootMotorStopped = true;
        footDriveSpeed = 0;
        WaitingforReconnect = true;
    }
    
    if (PS3NavDome->PS3NavigationConnected) 
    {

        if (criticalFaultDetectDome())
        {
           //We have a fault condition that we want to ensure that we do NOT process any controller data!!!
           printOutput();
           return false;
        }
    }
    
    return true;
}

// =======================================================================================
//          Print Output Function
// =======================================================================================

void printOutput()
{
    if (output != "")
    {
        if (Serial) Serial.println(output);
        output = ""; // Reset output string
    }
}
