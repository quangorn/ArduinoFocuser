// myfocuserPro ULN2003-MT
// ONLY FOR USE WITH 28BYJ-48 AND ULN2003 DRIVER
// MINIMAL + TEMP ONLY, Nano with ULN2003 driver and LEDS for IN/OUT
// NO SUPPORT FOR LCD
// NO SUPPORT FOR PUSH BUTTONS
// TEMPERATURE PROBE
//
// (c) Copyright Robert Brown 2014-2017. All Rights Reserved.
// The schematic, code and ideas for myFocuser are released into the public domain. Users are free to implement these but
// may NOT sell this project or projects based on this project for commercial gain without express written permission
// granted from the author. Schematics, Code, Firmware, Ideas, Applications, Layout are protected by Copyright Law.
// Permission is NOT granted to any person to redistribute, market, manufacture or sell for commercial gain the myFocuser
// products, ideas, circuits, builds, variations and units described or discussed herein or on this site.
// Permission is granted for personal and Academic/Educational use only.

// v1.70 31102017
// Fix errors in timer overflows

// v1.69 29072017
// Changes for compatibility with IDE v1.6.8
// Changes to Dallas Temperature Code and EEPROM and HalfStepper files (where included)
// Minor fixes

// v1.68_ULN2003_MT 14042017
// MT Version created for ULN2003

// v1.68_ULN2003_M 02112016
// Fix errors related to motorspeed setting and motor delays

// v1.67_ULN2003_M 28092016
// Fix for small values of stepsize between 0-1

// v1.66_ULN2003_M 11092016
// Fix to work with APT 3.13
// Fix incorrect logic in handling of GT temperature command for moonlite

// v1.63_ULN2003_M 04082016
// Small number of fixes related to LED's and PB code

// v1.62_ULN2003_M 26062016
// Fix error when disable/enable display

// v1.61_ULN2003_M 07062016
// Fix error when requesting MaxIncrement

// v1.60_ULN2003_M 14052016
// Tweaks to improve response
// Remove updatelcd when moving
// Add lcdupdateonmove, set and get
// Add LCD update when moving
// Remove tempwaitdelay - wait delay when requesting temperature
// Focuser remembers temperature mode (C/F) setting
// Focuser remembers temperature resolution setting
// Add get and set temperature resolution (9-12)
// Add get/set Motorspeed change and threshold setting
// Removed numwrites
// Removed :XY to free up space
// Support reset of focuser defaults from Windows App
// Save current focuser settings to EEPROM support for Windows App/
// Get motorspeed
// Set motorspeedchange value (threshold) when moving - switches to slowspeed when nearing destination
// Return motorspeedchange value for which stepper slows down at end of its move
// Enable/Disable motorspeedchange when moving
// Get status of motorspeedchange enabled/disabled
// Add Set/Get Step size
// Add Stepsize enabled/disabled
//
// For use with Windows App v1750 or greater
// For use wirh ASCOM driver v1600 or greater

// v1.57_ULN2003_M 18042016
// Major changes to temperature code
// Added commands to reset Arduino

// v1.56_ULN2003_M 12042016
// Support for different serial baud rates

// v1.53_ULN2003_M 26012016
// Added extra LCD page to display temperature and maxsteps
// Delay between LCD pages is user configurable and remembered between sessions
// This new feature only works with the latest application software and ASCOM drivers
// fixed bug in setting temperrorwaitdelay "SE" - was being set to 20,000 instead of 2,000
// Added GX and SX to handle updatedisplayintervalNotMoving

// v1.52_ULN2003_M 14122015
// minor bug fixes and removal of unneccessary code (All versions)
// motorspeed values verified/changed in some versions

// v1.51_ULN2003_M 15112015
// major bug fixes

// v1.50_ULN2003_M 13112015
// Restored dummy send on startup to communicate with moonlite focuser

// v1.49_ULN2003_M 09112015
// changes to DS18B20 precision and gettemp routines, fixes to IN/OUT/Buzzer LEDS
// added commands for display in celsius or fahrenheit (:SC# and :SF#)
// changed lcd display to handle C/F as well as increased step mode 64/128

// v1.48_ULN2003_M 24102015
// MaxSteps limit now 65000
// get MaxIncrement returns MaxIncrement
//
// v1.41_ULN2003_M 28052015
// Minor fixes
//
// v1.40_ULN2003_M 25052015
// On a :FQ (Halt), will write position to EEPROM after delay (in case user turns off focuser later on without any further moves)
// On powerup, both LED IN/OUT now light up with Buzzer On to indicate boot up cycle
//
// v1.38_ULN2003_M 27032015
// changed command :XY# to dump additional parameters
// changed motorSpeed values to get better differences between slow, med and fast
// changes to processcmd() SN, SP, PH, FQ, FG
//
// v1.35_ULN2003_M 22032015
// Halfsteps, Reverse Direction,  and CoilPwr now saved in EEPROM to allow for manual operation
// Changes required to startup and also processcmd (to ensure eeprom write)
//
// v1.34_ULN2003_M 15032015
// Test changes for INDI moonlite focuser
// Added gotonewposition variable, used with SN followed by FG (Moonlite)
// Problem is that SN is causing focuser to immediately move, its not waiting for FG
// now FG sets this flag to true, and when move is complete it is reset to false
// the moving condition in the main loop now also uses this flag as a condition to move
// Added GC for moonlite
// Changed maxFocuserLimit to 60000 for be compatible with moonlite
//
// v1.33_ULN2003_M 19022015
// changed Maxsteps and FocuserPosition to type long - On ,net and c# integers are 32 bit so support
// values greater than 32767. Changes maxFocuserLimit to 64000
//
// v1.31_ULN2003_M 05022015
// add stepdelay variable for moonlite protocol support
//
// v1.30_ULN2003_M 03022015
// changes to processcmd(), refined :POXX# to do temperature offset calibration for Moonlite protocol
// moved :GB to top of command processing to respond quicker
// added float tempoffsetval
//
// v1.29_ULN2003_M 03012014
// code change to ensure currentaddr does not end up with invalid value if not initialized
//
// v1.28_ULN2003_M 31122014
// Minor changes to motorSpeed
// added protocol to set motorSpeed via serial command
//
// v1.27_ULN2003_M 18122014
// Minor bug fix to analogWrite() statements
//
// v1.26_ULN2003_M 13122014
// In GF, Get Firmware version, return program name and version number (ASCOM uses GV)
//
// v1.25_ULN2003_M 10122014
// Modified driver code to write values to EEPROM on SP and SM commands
// At present code only updates 30s after a move. Using the Set() commands for
// position and maxStep does not initiate a move so these would not be saved
// unless there was a move() command that followed before shutdown/power-off
// Doubtful whether user would ever actually alter these parameters like this
// scenario describes, however included just in case
//
// v1.24_ULN2003_M 01122014
// Added GS/SS to work with ONE common Windows APP v1.25 and above
//
// v1.23_ULN2003_Minimal 22112014
// Decided to implement vars into EEPROM, require changes also to ASCOM Driver and Windows Apps
// NOTE: Designed to work with ATMEGA328P board and 1KB EEPROM, see define for EEPROMSIZE and
// change if not using ATMEGA328P
// implemented a 10s delay after a MOVE before EEPROM is updated (if no move then no update)
// only focuser position and maxsteps are saved in EEPROM
// coilpwr, stepmode/halfsteps and direction are saved by the application and sent to controller on connect()
//
// v1.22_ULN2003_Minimal 20112014
// fixed error in GZ
//
// v1.21_ULN2003_Minimal 26102104
// Compatible with myFocuser ASCOM driver
// Compatible with myFocuser Application
// Compatible with Moonlite ASCOM driver and application
//
// based on some portions of original code by Dave Wells, and orly.andico@gmail.com (moonlite focuser example)
// heavily modified by RBB to work with my stepper 28092014 motorPins[4] = {3,5,6,4};

#include <Arduino.h>
#include <Stepper.h>                  // needed for stepper motor
#include <OneWire.h>                  // needed for DS18B20 temperature probe
#include "EEPROM.h"                   // needed for EEPROM
#include "eepromanything.h"           // needed for EEPROM
#include "DallasTemperature.h"        // needed for DS18B20 temperature probe

#define EEPROMSIZE 1024               // ATMEGA328P 1024 EEPROM
#define SerialPortSpeed 9600

// these are stored in EEPROM - all variables in a structure
struct config_t {
  int validdata;       // if this is 99 then data is valid
  long fposition;       // last focuser position
  long maxstep;         // max steps
  int stepmode;                  // indicates stepmode, full, half, 1/4, 1/8. 1/16. 1/32 [1.2.4.8.16.32]
  int ds18b20resolution;         // resolution of DS18B20 temperature probe
  double stepsize;               // the step size in microns, ie 7.2
  boolean ReverseDirection;      // reverse direction
  boolean coilPwr;               // coil pwr
  boolean tempmode;              // temperature display mode, Celcius=1, Fahrenheit=0
  boolean stepsizeenabled;       // if true, controller returns step size
} myfocuser;

int datasize;                 // will hold size of the struct myfocuser - 6 bytes
int nlocations;               // number of storage locations available in EEPROM
int currentaddr;              // will be address in eeprom of the data stored
boolean writenow;             // should we update values in eeprom
boolean found;                // did we find any stored values?
long previousMillis = 0L;     // used as a delay whenever the EEPROM settings need to be updated
long myinterval = 10000L;     // interval in milliseconds to wait after a move before writing settings to EEPROM, 10s
int stepdelay = 0;            // used by moonlite
int tempcomp = 0;             // used by moonlite    
int tempoffsetval = 0;        // used by moonlite

const String programName = "myFocuser_ULN2003_MT";
const String programVersion = "1.7.0";

const int stepsPerRevolution = 2048; // there are 2048 steps for one rotation of the 28BYJ-48 shaft
// initialize the stepper library on pins 4 (IN1), 5 (IN2), 6 (IN3), 7 (IN4) (IN1, IN3, IN4, IN2)
Stepper mystepper(stepsPerRevolution, A4, A2, A1, A3);
// note that the stepper does not release current to the coils after a step so it gets hot
// the code below used by clearoutputs() releases the stepper after a step preventing it from getting hot
int motorPins[] = { A4, A3, A2, A1 };  // used to disable the output so it does not overheat
// motor speeds
// for 28BYJ-48 set Lo=1, Hi=2, Default=2
const int    motorSpeedSlowRPM = 5;
const int    motorSpeedMedRPM  = 10;
const int    motorSpeedFastRPM = 30;
// motorspeeddelay is a fixed value for ULN2003
int motorSpeedDelay = 3;                // the delay in millseconds between steps
int motorSpeedRPM = motorSpeedSlowRPM;  // the motorspeed RPM setting
int motorSpeed = 0;                     // 0=slow, 1=medium, 2=fast, default=slow on startup only
int savedmotorSpeed = motorSpeed;       // used to save original speed if slowing down when nearing target position

// define IN and OUT LEDS, associated with PB and stepper moves
#define bledIN A5
#define gledOUT A0
#define Buzzer 5

// Default initial positions if not set/overriden by Ascom Driver or Winapp
long currentPosition = 5000L;   // current position
long targetPosition = 5000L;    // target position
long maxFocuserLimit = 65000L;  // arbitary focuser limit
long maxSteps = 10000L;         // maximum position of focuser
long maxIncrement = 1000L;      // maximum number of steps permitted in one move
long minimumPosition = 0L;      // minimum position to avoid focuser damage
boolean gotonewposition = false;  // used by moonlite after an SN command followed by a FG

char inChar;
boolean isMoving = false;        // is the motor currently moving
long pos;

#define MAXCOMMAND 20
char mycmd[MAXCOMMAND];
char param[MAXCOMMAND];
char line[MAXCOMMAND];
int eoc = 0;    // end of command
int idx = 0;    // index into command string

// DS18B20 info
// define temperature probe
#define ch1temp 10             // temperature probe on PB2, use 4.7k pullup
// define objects for temperature probes
OneWire oneWirech1(ch1temp);  // setup temperature probe 1
// Pass our oneWire reference to Dallas Temperature library
DallasTemperature sensor1(&oneWirech1); // probe
#define TEMP_PRECISION 10  // Set the DS18B20s precision to 0.25 of a degree 9=0.5, 10=0.25, 11=0.125, 12=0.0625
DeviceAddress tpAddress;   // used to send precision setting to specific sensor
int tprobe1 = 0;           // indicate if there is a probe attached to that channel
double ch1tempval;         // temperature value for probe
double lasttempval = 20.0; // holds previous temperature value - used if ismoving and if temp request < 10s apart
double starttempval;       //
long lasttempconversion = 0L;  // holds time of last conversion
long temprefreshrate = 5000l;  // refresh rate between temperature conversions unless an update is requested via serial command

int TSWTHRESHOLD = 200;           // position at which stepper slows down at it approaches home position
int motorspeedchange = 1;         // slowdown focuser speed as it approaches target?

void software_Reboot()
{
  // jump to the start of the program
  asm volatile ( "jmp 0");
}

// disable the stepper motor outputs
void clearOutput()
{
  // check to see what is selected, keep or release
  if ( !myfocuser.coilPwr )
  {
    for (int i = 0; i < 4; i++)
    {
      digitalWrite(motorPins[i], 0);
    }
  }
}

// Move stepper anticlockwise
void anticlockwise()
{
  if ( !myfocuser.ReverseDirection )
  {
    digitalWrite(gledOUT, 1 ); // indicate a pulse by lighting the green LED
    mystepper.step(-1);        // step the motor one step anticlockwise
    digitalWrite(gledOUT, 0 ); // turn LED off
  }
  else
  {
    digitalWrite(bledIN, 1 );  // indicate a pulse by lighting the blue LED
    mystepper.step(1);         // step the motor one step clockwise
    digitalWrite(bledIN, 0 );  // turn LED off
  }
}

// Move stepper clockwise
void clockwise()
{
  if ( !myfocuser.ReverseDirection )
  {
    digitalWrite(bledIN, 1 );  // indicate a pulse by lighting the blue LED
    mystepper.step(1);         // step the motor one step clockwise
    digitalWrite(bledIN, 0 );  // turn LED off
  }
  else
  {
    digitalWrite(gledOUT, 1 ); // indicate a pulse by lighting the green LED
    mystepper.step(-1);        // step the motor one step anticlockwise
    digitalWrite(gledOUT, 0 ); // turn LED off
  }
}

// set the microstepping mode
void setstepmode()
{
  // ULN2003 does not support stepping
  // reset to full steps
  myfocuser.stepmode = 1;
}

// Serial Commands
void processCommand()
{
  memset(mycmd, 0, MAXCOMMAND);
  memset(param, 0, MAXCOMMAND);
  int len = strlen(line);
  if (len >= 2)
  {
    strncpy(mycmd, line, 2);
  }
  if (len > 2)
  {
    strncpy(param, line + 2, len - 2);
  }

  memset(line, 0, MAXCOMMAND);
  eoc = 0;
  idx = 0;

  // :SF# set fullstep mode
  if (!strcasecmp( mycmd, "SF"))
  {
    // ignore full steps only
  }

  // :SH# set halfstep mode
  else if (!strcasecmp( mycmd, "SH"))
  {
    // ignore, always full steps for ULN2003
  }

  // :GH# whether half-step is enabled or not, always return "00"
  else if (!strcasecmp( mycmd, "GH"))
  {
    Serial.print("00#");
  }

  // :SSxxx# set stepmode
  // myfocuser command
  else if (!strcasecmp( mycmd, "SS"))
  {
    myfocuser.stepmode = 1;    // ULN2003 has full steps only
  }

  // :GS# get stepmode
  // myFocuser Command
  else if (!strcasecmp(mycmd, "GS"))
  {
    char tempString[6];
    sprintf(tempString, "%02X", myfocuser.stepmode);
    Serial.print(tempString);
    Serial.print("#");
  }

  // :MS# set motorSpeed - time delay between pulses, acceptable values are 00, 01 and 02 which
  // correspond to a slow, med, high
  // myFocuser Command
  else if (!strcasecmp(mycmd, "MS"))
  {
    int pos = decstr2int(param);
    if ( pos == 0 )
    {
      motorSpeedRPM = motorSpeedSlowRPM;                    // slow
      motorSpeed = 0;
    }
    else if ( pos == 1 )
    {
      motorSpeedRPM = motorSpeedMedRPM;                     // medium
      motorSpeed = 1;
    }
    else if (pos == 2 )
    {
      motorSpeedRPM = motorSpeedFastRPM;                    // fast
      motorSpeed = 2;
    }
    else
    {
      motorSpeedRPM = motorSpeedSlowRPM;
      motorSpeed = 0;
    }
    savedmotorSpeed = motorSpeed;           // remember the speed setting
    mystepper.setSpeed(motorSpeedRPM);      // update the motor speed
  }

  // :GP# get the current focuser position
  else if (!strcasecmp(mycmd, "GP"))
  {
    char tempString[6];
    sprintf(tempString, "%04X", (unsigned int) currentPosition);
    Serial.print(tempString);
    Serial.print("#");
  }

  // :GI# motor is moving - 1 if moving, 0 otherwise
  else if (!strcasecmp(mycmd, "GI"))
  {
    if (isMoving ) {
      Serial.print("01#");
    }
    else {
      Serial.print("00#");
    }
  }

  // :GT# get the current temperature - moonlite compatible
  else if (!strcasecmp( mycmd, "GT"))
  {
    char tempString[6];
    
    // Send the command to get temperatures from DS18B20 probe
    if ( isMoving == false )   // if focuser is not moving
    {
      // if there is a temperature probe
      if ( tprobe1 == true )
      {
        gettemp();
      }
      else
      {
        // there is not a probe so just return 20
        ch1tempval = 20.0;
      }
    }
    else
    {
      // focuser is moving, send back last temp value
    }
    // change v166
    int tpval = (ch1tempval * 2);
    sprintf(tempString, "%04X", (int) tpval);
    Serial.print(tempString);;
    Serial.print("#");
  }

  // :GZ# get the current temperature
  else if (!strcasecmp( mycmd, "GZ"))
  {
    // change v166
    if ( isMoving == false )
    {
      // is focuser moving
      if ( tprobe1 == true )
      {
        // not moving so do a temperature conversion
        gettemp();
      }
      else
      {
        // there is not a probe so just return 20
        ch1tempval = 20.0;
      }
    }
    else
    {
        // focuser is moving, send back last temp value
    }
    char tempstr[8];
    dtostrf(ch1tempval, 4, 3, tempstr);
    String tempretval(tempstr);
    Serial.print(tempretval);
    Serial.print("#");
  }

  // :GV# firmware value Moonlite
  else if (!strcasecmp(mycmd, "GV"))
  {
    Serial.print("10#");
  }

  // :GF# firmware value
  // myFocuser Command
  else if (!strcasecmp(mycmd, "GF"))
  {
    Serial.println(programName);
    Serial.print(programVersion);
    Serial.print("#");
  }

  // :GM# get the MaxSteps
  // myFocuser Command
  else if (!strcasecmp(mycmd, "GM"))
  {
    char tempString[6];
    sprintf(tempString, "%04X", (unsigned int) maxSteps);
    Serial.print(tempString);
    Serial.print("#");
  }

  // :GY# get the maxIncrement - set to MaxSteps
  // myFocuser Command
  else if (!strcasecmp(mycmd, "GY"))
  {
    char tempString[6];
    sprintf(tempString, "%04X", (unsigned int) maxIncrement);
    Serial.print(tempString);
    Serial.print("#");
  }

  // :GO# get the coilPwr setting
  // myFocuser Command
  else if (!strcasecmp(mycmd, "GO"))
  {
    String tempString;
    if ( myfocuser.coilPwr )
      tempString = "01#";
    else
      tempString = "00#";
    Serial.print(tempString);
  }

  // :GR# get the Reverse Direction setting
  // myFocuser Command
  else if (!strcasecmp(mycmd, "GR"))
  {
    String tempString;
    if ( myfocuser.ReverseDirection )
      tempString = "01#";
    else
      tempString = "00#";
    Serial.print(tempString);
  }

  // :MR# get Motor Speed
  // myfocuser command
  else if (!strcasecmp( mycmd, "MR"))
  {
    Serial.print(motorSpeed);
    Serial.print("#");
  }

  // :MTxxx# set the MotorSpeed Threshold
  // myfocuser command
  else if (!strcasecmp( mycmd, "MT"))
  {
    int pos = decstr2int(param);
    if ( pos < 50 )
    {
      pos = 50;
    }
    else if ( pos > 200 )
    {
      pos = 200;
    }
    TSWTHRESHOLD = pos;
  }

  // :MU# Get the MotorSpeed Threshold
  // myfocuser command
  else if (!strcasecmp( mycmd, "MU"))
  {
    Serial.print(TSWTHRESHOLD);
    Serial.print("#");
  }

  // :MVx#        None      Set Enable/Disable motorspeed change when moving
  else if (!strcasecmp( mycmd, "MV"))
  {
    int pos = decstr2int(param);
    motorspeedchange = pos;
  }

  // :MW#         xxx#      Get if motorspeedchange enabled/disabled
  else if (!strcasecmp( mycmd, "MW"))
  {
    Serial.print(motorspeedchange);
    Serial.print("#");
  }

  // :MX#         None      Save settings to EEPROM
  else if (!strcasecmp( mycmd, "MX"))
  {
    // copy current settings and write the data to EEPROM
    myfocuser.validdata = 99;
    myfocuser.fposition = currentPosition;
    myfocuser.maxstep = maxSteps;
    EEPROM_writeAnything(currentaddr, myfocuser);    // update values in EEPROM
    writenow = false;
  }

  // :SOxx# set the coilPwr setting
  // myFocuser Command
  else if (!strcasecmp(mycmd, "SO"))
  {
    int pos = decstr2int(param);
    if ( pos == 0 )
      myfocuser.coilPwr = false;
    else
      myfocuser.coilPwr = true;
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }

  // :SRxx# set the Reverse Direction setting
  // myFocuser Command
  else if (!strcasecmp(mycmd, "SR"))
  {
    int pos = decstr2int(param);
    if ( pos == 0 )
      myfocuser.ReverseDirection = false;
    else
      myfocuser.ReverseDirection = true;
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }

  // :DMx# set displaystate C or F
  else if ( !strcasecmp( mycmd, "DM"))
  {
    int pos = decstr2int(param);
    if ( pos > 0 )
    {
      myfocuser.tempmode = false;
      writenow = true;             // updating of EEPROM ON
      previousMillis = millis();   // start time interval
    }
    else
    {
      myfocuser.tempmode = true;
      writenow = true;             // updating of EEPROM ON
      previousMillis = millis();   // start time interval
    }
  }

  // :FG# initiate a move to the target position
  else if (!strcasecmp( mycmd, "FG"))
  {
    gotonewposition = true;
    isMoving = true;
  }

  // :FQ# stop a move - HALT
  else if (!strcasecmp( mycmd, "FQ"))
  {
    gotonewposition = false;
    isMoving = false;
    targetPosition = currentPosition;
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }

  // :PH# home the motor, hard-coded, ignore parameters
  // not implemented in INDI driver
  else if (!strcasecmp( mycmd, "PH"))
  {
    gotonewposition = true;
    isMoving = true;
    targetPosition = 0;
  }

  // :GN# get the new motor position (target)
  // not implemented in INDI driver
  else if (!strcasecmp( mycmd, "GN"))
  {
    char tempString[6];
    sprintf(tempString, "%04X", (unsigned int) targetPosition);
    Serial.print(tempString);
    Serial.print("#");
  }

  // :SPxxxx# set current position to received position - no move SPXXXX
  // in INDI driver, only used to set to 0 SP0000 in reset()
  else if (!strcasecmp( mycmd, "SP"))
  {
    pos = hexstr2long(param);
    if ( pos > maxSteps )
      pos = maxSteps;
    if ( pos < 0 )
      pos = 0;
    currentPosition = pos;
    targetPosition = pos;
    // signal that the focuser position has changed and should be saved to eeprom
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
    gotonewposition = false;
    isMoving = false;
  }

  // :SNxxxx# set new target position SNXXXX - this is a move command
  // but must be followed by a FG command to start the move
  else if (!strcasecmp( mycmd, "SN"))
  {
    pos = hexstr2long(param);
    if ( pos > maxSteps )
      pos = maxSteps;
    if ( pos < 0 )
      pos = 0;
    targetPosition = pos;
    gotonewposition = false;
    isMoving = false;
  }

  // :GD# get the current motor step delay, only values of 02, 04, 08, 10, 20
  // not used so just return 02
  else if (!strcasecmp( mycmd, "GD"))
  {
    char tempString[6];
    sprintf(tempString, "%02X", stepdelay);
    Serial.print(tempString);
    Serial.print("#");
  }

  // :SDxx# set step delay, only acceptable values are 02, 04, 08, 10, 20 which
  // correspond to a stepping delay of 250, 125, 63, 32 and 16 steps
  // per second respectively. Moonlite only
  else if (!strcasecmp( mycmd, "SD"))
  {
    pos = hexstr2long(param);
    stepdelay = (int) pos;
  }

  // :SCxx# set temperature co-efficient XX
  else if (!strcasecmp( mycmd, "SC"))
  {
    // do nothing, ignore
    pos = hexstr2long(param);
    tempcomp = (int) pos;
  }

  // :GC# get temperature co-efficient XX
  else if (!strcasecmp( mycmd, "GC"))
  {
    // do nothing, ignore
    char tempString[6];
    sprintf(tempString, "%02X", tempcomp);
    Serial.print(tempString);
    Serial.print("#");
  }

  // + activate temperature compensation focusing
  else if (!strcasecmp( mycmd, "+"))
  {
    // ignore
  }

  // - disable temperature compensation focusing
  else if (!strcasecmp( mycmd, "-"))
  {
    // ignore
  }

  // :PO# temperature calibration offset POXX in 0.5 degree increments (hex)
  else if (!strcasecmp( mycmd, "PO"))
  {
    // Moonlite only
    // this adds/subtracts an offset from the temperature reading in 1/2 degree C steps
    // FA -3, FB -2.5, FC -2, FD -1.5, FE -1, FF -.5, 00 0, 01 0.5, 02 1.0, 03 1.5, 04 2.0, 05 2.5, 06 3.0
    tempoffsetval = 0.0;
    // param is a char []
    String parm1 = String(param);
    if ( parm1 == "FA" )
      tempoffsetval = -3.0;
    else if ( parm1 == "FB")
      tempoffsetval = -2.5;
    else if ( parm1 == "FC")
      tempoffsetval = -2.0;
    else if ( parm1 == "FD")
      tempoffsetval = -1.5;
    else if ( parm1 == "FE")
      tempoffsetval = -1.0;
    else if ( parm1 == "FF")
      tempoffsetval = -0.5;
    else if ( parm1 == "00")
      tempoffsetval = 0.0;
    else if ( parm1 == "01")
      tempoffsetval = 0.5;
    else if ( parm1 == "02")
      tempoffsetval = 1.0;
    else if ( parm1 == "03")
      tempoffsetval = 1.5;
    else if ( parm1 == "04")
      tempoffsetval = 2.0;
    else if ( parm1 == "05")
      tempoffsetval = 2.5;
    else if ( parm1 == "06")
      tempoffsetval = 3.0;
  }

  // :SMxxx# set new maxSteps position SMXXXX
  // myfocuser command
  else if (!strcasecmp( mycmd, "SM"))
  {
    pos = hexstr2long(param);
    if ( pos > maxFocuserLimit )
      pos = maxFocuserLimit;
    // avoid setting maxSteps too low
    if ( pos < 1000 )
      pos = 1000;
    // for NEMA17 at 400 steps this would be 5 full rotations of focuser knob
    // for 28BYG-28 this would be less than 1/2 a revolution of focuser knob
    maxSteps = pos;
    // check maxIncement in case its larger
    if ( maxIncrement > maxSteps )
      maxIncrement = maxSteps;
    // signal that the focuser position has changed and should be saved to eeprom
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }

  // :SYxxxx# set new maxIncrement SYXXXX
  // myfocuser command
  else if (!strcasecmp( mycmd, "SY"))
  {
    pos = hexstr2long(param);
    // ignore
    maxIncrement = maxSteps;
  }

  // :DSx# disable or enable the display setting
  else if (!strcasecmp( mycmd, "DS"))
  {
    // ignore, no display
  }

  // :DG# get display state on or off
  else if (!strcasecmp( mycmd, "DG"))
  {
    Serial.print("00#");
  }

  // :GXxxxxx#		      get the time that an LCD screen is displayed for (in milliseconds, eg 2500 = 2.5seconds
  else if ( !strcasecmp( mycmd, "GX"))
  {
    char tempString[12];
    sprintf(tempString, "%04X", 2000);
    Serial.print(tempString);
    Serial.print("#");
  }

  // :SXxxxx#	None		Set updatedisplayNotMoving (length of time an LCD page is displayed for in milliseconds
  else if ( !strcasecmp( mycmd, "SX"))
  {
    // ignore, no display
  }

  // :TA#  Reboot Arduino
  else if ( !strcasecmp( mycmd, "TA"))
  {
    software_Reboot();
  }

  // :PS    Set temperature precision (9-12 = 0.5, 0.25, 0.125, 0.0625)
  else if ( !strcasecmp( mycmd, "PS"))
  {
    int precision = decstr2int(param);
    if ( tprobe1 == 1 )
    {
      sensor1.setResolution( tpAddress, precision );
      myfocuser.ds18b20resolution = precision;        // save setting in EEPROM
      writenow = true;             // updating of EEPROM ON
      previousMillis = millis();   // start time interval
    }
    else
    {
      // no probe, set to default
      myfocuser.ds18b20resolution = TEMP_PRECISION;
    }
  }

  // :PG    Get temperature precision (9-12)
  else if ( !strcasecmp( mycmd, "PG"))
  {
    if ( tprobe1 == 1 )
    {
      myfocuser.ds18b20resolution = sensor1.getResolution(tpAddress);
      writenow = true;             // updating of EEPROM ON
      previousMillis = millis();   // start 30s time interval
    }
    Serial.print(myfocuser.ds18b20resolution);
    Serial.print("#");
  }

  // :PMxx#    None			set update of position on lcd when moving (00=disable, 01=enable)
  else if ( !strcasecmp( mycmd, "PM"))
  {
    // ignore
  }

  // :PN#	      xx#			get update of position on lcd when moving (00=disable, 01=enable)
  else if ( !strcasecmp( mycmd, "PN"))
  {
    Serial.print("00#");
  }

  // :PZxx#	  None			Set the return of user specified stepsize to be OFF - default (0) or ON (1)
  else if ( !strcasecmp( mycmd, "PZ"))
  {
    int pos = decstr2int(param);
    if ( pos == 1 )
    {
      myfocuser.stepsizeenabled = true;
    }
    else   // any other value please disable
    {
      myfocuser.stepsizeenabled = false;
    }
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }

  // :PPxxxx#  None			Set the step size value - double type, eg 2.1
  else if ( !strcasecmp( mycmd, "PP"))
  {
    // convert param to float
    String str = param;
    str = str + "";      // add end of string terminator
    double tempstepsize = (double) str.toFloat();
    if ( tempstepsize < 0 )
    {
      tempstepsize = 0;
    }
    myfocuser.stepsize = tempstepsize;
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();   // start time interval
  }

  // :PQ#	  None			Get if stepsize is enabled in controller (true or false, 0/1)
  else if ( !strcasecmp( mycmd, "PQ"))
  {
    if (myfocuser.stepsizeenabled == true)
      Serial.print("1#");
    else
      Serial.print("0#");
  }

  // :PR#	  xxxxx#		Get step size in microns (if enabled by controller)
  else if ( !strcasecmp( mycmd, "PR"))
  {
    Serial.print(myfocuser.stepsize);
    Serial.print("#");
  }

  // :FM#    x#      Get Display temp mode (Celsius=0, Fahrenheit=1)
  else if ( !strcasecmp( mycmd, "FM"))
  {
    if (myfocuser.tempmode == true)  // when true display is in celsius
      Serial.print("0#");
    else
      Serial.print("1#");
  }

  // :XY# troubleshooting only - print currentaddr value, use in serial monitor mode is best
  else if (!strcasecmp( mycmd, "XY"))
  {
    Serial.print("-#");
  }

  // troubleshooting only - reset focuser defaults
  // myFocuser Command
  else if (!strcasecmp(mycmd, "XZ"))
  {
    currentaddr = 0;
    myfocuser.validdata = 99;
    myfocuser.fposition = 5000L;
    myfocuser.maxstep = 10000L;
    myfocuser.stepmode = 1;
    myfocuser.stepsize = 5.4;
    myfocuser.ReverseDirection = false;
    myfocuser.coilPwr = true;
    myfocuser.tempmode = true; // celsius
    myfocuser.ds18b20resolution = TEMP_PRECISION;
    myfocuser.stepsizeenabled = false;;
    // now write the data to EEPROM
    EEPROM_writeAnything(currentaddr, myfocuser);    // update values in EEPROM
  }
}

long hexstr2long(char *line)
{
  long ret = 0;

  ret = strtol(line, NULL, 16);
  return (ret);
}

// convert string to int
int decstr2int(char *line)
{
  int ret = 0;
  String Str(line);

  ret = Str.toInt();
  return ret;
}

// find the address of the DS18B20 sensor probe
void findds18b20address()
{
  // look for probes
  // Search the wire for address
  if (sensor1.getAddress(tpAddress, 0))
  {
    tprobe1 = 1;  // there is a probe1
  }
  // device address is now saved in tpAddress
}

void gettemp()
{
  sensor1.requestTemperatures();
  delay(700 / (1 << (12 - myfocuser.ds18b20resolution))); // should enough time to wait
  // get channel 1 temperature, always in celsius
  double result = sensor1.getTempCByIndex(0);
  // sometimes sensor returns -127, not sure why, so check and use last reading if this is the case
  // range of sensor is -55c to +125c
  // limit to values of -40c to +80c (-40f to 176f)
  int tempval = (int) result;
  if ( tempval < -40 )
    ch1tempval = lasttempval;
  else if ( tempval > 80 )
    ch1tempval = lasttempval;
  else
  {
    ch1tempval = result;
    lasttempval = ch1tempval;         // remember last reading
  }
}

// Setup
void setup()
{
  // initialize serial
  Serial.begin(SerialPortSpeed);

  // turn ON the Buzzer - provide power ON beep
  pinMode(Buzzer, OUTPUT);
  digitalWrite( Buzzer, 1);
  // turn ON both LEDS as power on cycle indicator
  pinMode( bledIN, OUTPUT);
  pinMode( gledOUT, OUTPUT);
  digitalWrite( bledIN, 1 );
  digitalWrite( gledOUT, 1 );

  // start temperature sensor DS18B20
  ch1tempval  = 20.0;
  tprobe1 = 0;        // set probe indicator NOT FOUND

  eoc = 0;
  idx = 0;
  isMoving = false;
  gotonewposition = false;
  memset(line, 0, MAXCOMMAND);

  currentaddr = 0;    // start at 0 if not found later
  found = false;
  writenow = false;
  datasize = sizeof( myfocuser );    // should be 14 bytes
  nlocations = EEPROMSIZE / datasize;  // for AT328P = 1024 / datasize = 73 locations

  for (int lp1 = 0; lp1 < nlocations; lp1++ )
  {
    int addr = lp1 * datasize;
    EEPROM_readAnything( addr, myfocuser );
    // check to see if the data is valid
    if ( myfocuser.validdata == 99 )
    {
      // data was erased so write some default values
      currentaddr = addr;
      found = true;
    }
  }
  if ( found == true )
  {
    // set the focuser back to the previous settings
    // done after this in one hit
    // mark current eeprom address as invalid and use next one
    // each time focuser starts it will read current storage, set it to invalid, goto next location and
    // write values to there and set it to valid - so it doesnt always try to use same locations over and
    // over and destroy the eeprom
    // using it like an array of [0-nlocations], ie 100 storage locations for 1k EEPROM
    EEPROM_readAnything( currentaddr, myfocuser );
    myfocuser.validdata = 0;
    EEPROM_writeAnything(currentaddr, myfocuser);    // update values in EEPROM

    // goto next free address and write data
    currentaddr += datasize;
    // bound check the eeprom storage and if greater than last index [0-EEPROMSIZE-1] then set to 0
    if ( currentaddr >= (nlocations * datasize) ) currentaddr = 0;

    myfocuser.validdata = 99;
    EEPROM_writeAnything(currentaddr, myfocuser);    // update values in EEPROM
  }
  else
  {
    // set defaults because not found
    myfocuser.validdata = 99;
    myfocuser.fposition = 5000L;
    myfocuser.maxstep = 10000L;
    myfocuser.stepmode = 1;
    myfocuser.ReverseDirection = false;
    myfocuser.coilPwr = true;
    myfocuser.tempmode = true;    // default tempmode to celsius if not found
    myfocuser.ds18b20resolution = TEMP_PRECISION;      // 10 bit = 0.25 degrees
    myfocuser.stepsizeenabled = false;
    myfocuser.stepsize = 5.4;
    // now write the data to EEPROM
    EEPROM_writeAnything(currentaddr, myfocuser);    // update values in EEPROM
  }

  // Set focuser defaults from saved values in EEPROM.
  currentPosition = myfocuser.fposition;
  targetPosition = myfocuser.fposition;
  maxSteps = myfocuser.maxstep;

  // start temperature sensor DS18B20
  // this must be done after the eeprom settings are retreived into myfocuser struct !!!!!!!!
  ch1tempval  = 20.0;
  lasttempval = 20.0;
  tprobe1 = 0;        // set probe indicator NOT FOUND
  sensor1.begin();    // start the temperature sensor1
  sensor1.getDeviceCount();    // should return 1 if probe connected
  findds18b20address();
  if ( tprobe1 == 1 )
  {
    sensor1.setResolution( tpAddress, myfocuser.ds18b20resolution );
    gettemp();
  }

  setstepmode();

  // remember speed setting
  motorSpeedRPM = motorSpeedSlowRPM;  // the motorspeed RPM setting
  motorSpeed = 0;                     // 0=slow, 1=medium, 2=fast, default=slow on startup only
  savedmotorSpeed = motorSpeed;       // used to save original speed if slowing down when nearing target position
  mystepper.setSpeed(motorSpeedRPM);

  TSWTHRESHOLD = 200;                 // position at which stepper slows down at it approaches home position
  motorspeedchange = 1;

  writenow = false;

  // turn off the IN/OUT LEDS and BUZZER
  digitalWrite( bledIN, 0 );
  digitalWrite( gledOUT, 0 );
  digitalWrite( Buzzer, 0);
}

// Main Loop
void loop()
{
  // Move the position by a single step if target is different to current position
  if ((targetPosition != currentPosition) && (gotonewposition == true))
  {
    // Slow down if approaching home position
    if ( motorspeedchange == 1 )
    {
      // Slow down if approaching home position
      int nearinghomepos = currentPosition - targetPosition;
      nearinghomepos = abs(nearinghomepos);
      if ( nearinghomepos < TSWTHRESHOLD )
      {
        motorSpeed = 0;                        // slow
        motorSpeedRPM = motorSpeedSlowRPM;
        mystepper.setSpeed(motorSpeedSlowRPM);     // send command to set stepper speed
      }
    }

    isMoving = true;
    writenow = true;             // updating of EEPROM ON
    previousMillis = millis();    // keep updating previousMillis whilst focuser is moving

    // Going Anticlockwise to lower position
    if (targetPosition < currentPosition)
    {
      anticlockwise();
      currentPosition--;    // might be ++?
    }

    // Going Clockwise to higher position
    if (targetPosition > currentPosition)
    {
      // do not need to check if > maximumPosition as its done when a target command is receieved
      clockwise();
      currentPosition++;
    }
    delay( motorSpeedDelay );  // required else stepper will not move
  }
  else
  {
    // focuser is NOT moving now, move is completed
    isMoving = false;
    gotonewposition = false;

    // reset motorSpeed
    motorSpeed = savedmotorSpeed;
    switch (motorSpeed)
    {
      case 0: motorSpeedRPM = motorSpeedSlowRPM; break;
      case 1: motorSpeedRPM = motorSpeedMedRPM; break;
      case 2: motorSpeedRPM = motorSpeedFastRPM; break;
      default: motorSpeedRPM = motorSpeedSlowRPM; break;
    }
    mystepper.setSpeed(motorSpeedRPM);

    // if there is a temperature probe
    if ( tprobe1 == 1)
    {
      long tempnow = millis();
      // see if the temperature needs updating - done automatically every 5s
      if ( ((tempnow - lasttempconversion) > temprefreshrate) || (tempnow < lasttempconversion))
      {
        lasttempconversion = tempnow;
        sensor1.requestTemperatures();
        delay(600 / (1 << (12 - myfocuser.ds18b20resolution))); // should enough time to wait
        // get channel 1 temperature, always in celsius
        double result = sensor1.getTempCByIndex(0);
        // sometimes sensor returns -127, not sure why, so check and use last reading if this is the case
        // range of sensor is -55c to +125c
        // limit to values of -40c to +80c (-40f to 176f)
        int tempval = (int) result;
        if ( tempval < -40 )
          ch1tempval = lasttempval;
        else if ( tempval > 80 )
          ch1tempval = lasttempval;
        else
        {
          ch1tempval = result;
          lasttempval = ch1tempval;         // remember last reading
        }
      } // end of if( tempnow .... )
    } // end of if( tprobe1 == 1 )

    // is it time to update EEPROM settings?
    if ( writenow == true )
    {
      // decide if we have waited 10s after the last move, if so, update the EEPROM
      long currentMillis = millis();
      if ( ((currentMillis - previousMillis) > myinterval) || (currentMillis < previousMillis) )
      {
        previousMillis = currentMillis;
        // copy current settings and write the data to EEPROM
        myfocuser.validdata = 99;
        myfocuser.fposition = currentPosition;
        myfocuser.maxstep = maxSteps;
        EEPROM_writeAnything(currentaddr, myfocuser);    // update values in EEPROM
        writenow = false;
      }
    }
    clearOutput(); // release the stepper coils to save power
  }
}

// SerialEvent occurs whenever new data comes in the serial RX.
void serialEvent()
{
  // : starts the command, # ends the command, do not store these in the command buffer
  // read the command until the terminating # character
  while (Serial.available() && !eoc)
  {
    inChar = Serial.read();
    if (inChar != '#' && inChar != ':')
    {
      line[idx++] = inChar;
      if (idx >= MAXCOMMAND)
      {
        idx = MAXCOMMAND - 1;
      }
    }
    else
    {
      if (inChar == '#')
      {
        eoc = 1;
        idx = 0;
        // process the command string when a hash arrives:
        processCommand();
        eoc = false;
      }
    }
  }
}
