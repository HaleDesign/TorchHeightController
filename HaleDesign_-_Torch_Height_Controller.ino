/*
    Torch Height Controller
    Copyright (C)2020  by Jeremiah Hale - HaleDesignTech - Principal Engineer

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

  Software Version: 1.0.0
  Compatible with THC Nextion Screen Firmware Version: 1.0.0

  Aim:
  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  Arduino based THC that reads 50:1 or 16:1 plasma voltage and send Up and Down signals to Plasma Torch Actuator to adjust voltage to target value.

  Description:
  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  It's important for a plasma arc to be stable and be a set height from the workpiece to be cut.
  The main reason for this is the plasma arc will cut a bevel on the side walls of the height it not set right or crash into the workpiece...
  This is because the plasma arc is not like a laser with straight edges but more like an egg.
  Making the problem worse is the fact the metal can warp and contort when a hot plasma arc cuts into it.
  Using the Arc Voltage is a good want to estimate the distance to the workpiece from the torch head.
  The unknown we are trying to solve for here is the Torch Height from the Workpiece.
  A proportional correlation is the longer the arc the higher the voltage.
  So, we can measure the plasma voltage and feed that into a PID Algorithm to calculate
  the torch height to change the voltage to a setpoint.
  It is unwise to measure the Arc Voltage Directly of the plasma torch because the levels there can be deadly.
  Most CNC ready Plasma Cutters on the market have 50:1 arc voltage dividers built right into the machine.
  If yours doesn't have this then you will need to do surgery and add a voltage divider circuit to your plasma cutter...
  Check the Technical Specs of your plasma cutter:
  Don't Die.


  Hardware:
  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  Arduino: Uno R3
  Nextion HMI Screen: NX4832T035_011
  Makerbase: MKS TMC2160-OC V1.0
  2 Fans(overkill I know): 40 x 40 x 10mm 4010 Brushless DC Cooling Fan 12v
  Aluminum Electrolytic Capacitor(optional): Nichicon USA1H010MDD1TE 1µF 50V Aluminum Electrolytic Capacitors Radial
  Input Connector: Cat5e Ethernet RJ-45 Keystone Jack
  Output Connector: Cat5e Ethernet RJ-45 Keystone Jack
  Power: 10v 5Amp DC barrel jack
  PC diagnostic: USB A - USB B

  Donate:
  https://www.patreon.com/HaleDesign

  More info:
  https://github.com/HaleDesign/TorchHeightController
  http://hdt.xyz

  3rd Party Software:
  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  FastPID Library
  by Mike Matera

  INSTALL
  The FastPIDPID Library is available in the Arduino IDE Library Manager

  DOWNLOADS
  PID Library
  Latest version on GitHub: https://github.com/mike-matera/FastPID
  Version Used: v1.3.0


  EasyNextionLibrary
  by Athanasios Seitanis
  contact: eithagta@gmail.com

  INSTALL
  The EasyNextionLibrary is available in the Arduino IDE Library Manager

  DOWNLOADS
  Download the latest release of EasyNextionLibrary.
  From: https://github.com/Seithan/EasyNextionLibrary
  Version Used: v1.0.4

  AccelStepper
  by Mike McMauley

  INSTALL
  The AccelStepper is available in the Arduino IDE Library Manager

  More info
  From: http://www.airspayce.com/mikem/arduino/AccelStepper/
  Version Used: v1.61.0


  Input:
  Arc Voltage, conservative P I D parameters, aggressive P I D parameters, Gap(amount away from setpoint for Agg/Con PID Settings), Arc Voltage Setpoint, ArcStablizeDelay, and Z axes bountray limits.

*/


#include <FastPID.h>              // Include PID Library     
#include <EasyNextionLibrary.h>   // Include EasyNextionLibrary
#include <EEPROMex.h>             // Include Custom Arudino EEPROM Lib Uno: 1kb EEPROM storage.
#include <AccelStepper.h>

// the variables to be using be the code below
EasyNex THCNex(Serial); // Create an object of EasyNex class with the name < TCHNex >
// Set as parameter the Serial you are going to use
// Default baudrate 9600


#define PLASMA_INPUT_PIN A1
#define STEP_PIN 3      // Direction
#define DIR_PIN 4       // Step
//MKS Drive board enable pin in 13
//No need to define because it uses the onboard LED on the Arduino Uno R3

// Define a stepper driver and the pins it will use
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

//Define Variables
double Input = 0;
double targetInput;
double gap;
double scale;
int threshold;
int currentGap;
uint32_t oldDelay;
uint32_t arcStabilizeDelay;
int SetPoint = 0;
int CalibrationOffset = 0;


//Specify the links and initial tuning parameters
float aggKp = 0.175, aggKi = 0.1, aggKd = 0.1;
float Kp = 0.075, Ki = 0.01, Kd = 0.01;
float Hz = 8;
int output_bits = 16;
bool output_signed = true;
bool alreadySetColor = false;

FastPID THCPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

// Set EEPROM Addresses for Setpoint saving
int addressPage1 = 10;
int addressPage2 = 20;
int addressPage3 = 30;
int addressPage4 = 40;
int addressPage5 = 50;
int addressPage6 = 60;
int addressGap = 70;
int addressThreshold = 80;
int addressDelay = 90;
int addressSteps = 100;
int addressCalibrate = 110;
int addressMaxpos = 120;
int addressMinpos = 130;
int addressAP = 200;
int addressAI = 300;
int addressAD = 400;
int addressCP = 500;
int addressCI = 600;
int addressCD = 700;
int addressScale = 800;

int defaultSetpoint = 10900;

int SetpointPage1 = 0;
int SetpointPage2 = 0;
int SetpointPage3 = 0;
int SetpointPage4 = 0;
int SetpointPage5 = 0;
int SetpointPage6 = 0;


int CurrentPageNumber = 0;
int SavedPage = 0;

//movement
int steps_per_mm = 200;
float pos = 0;
float adjpos = 0;
int minPos = -(40 * steps_per_mm);
int maxPos = (40 * steps_per_mm);
int moveAmt = 0;
uint8_t output = 0;

// the setup function runs once when you press reset or power the board
void setup()
{
  // Initialize digital pin LED_BUILTIN as an output.
  //This is used to enable the MKS driver board. Plus it flashes and flashes are cool.
  pinMode(LED_BUILTIN, OUTPUT);

  // Begin the object with a baud rate of 9600
  THCNex.begin();  // If no parameter was given in the begin(), the default baud rate of 9600 will be used
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //initialize the variables we're linked to
  // Load EEPROM Addresses for Setpoints or set defaults
  SetpointPage1 = EEPROM.readInt(addressPage1);
  if (SetpointPage1 == 0) {
    SetpointPage1 = defaultSetpoint;
  }

  SetpointPage2 = EEPROM.readInt(addressPage2);
  if (SetpointPage2 == 0) {
    SetpointPage2 = defaultSetpoint;
  }

  SetpointPage3 = EEPROM.readInt(addressPage3);
  if (SetpointPage3 == 0) {
    SetpointPage3 = defaultSetpoint;
  }

  SetpointPage4 = EEPROM.readInt(addressPage4);
  if (SetpointPage4 == 0) {
    SetpointPage4 = defaultSetpoint;
  }

  SetpointPage5 = EEPROM.readInt(addressPage5);
  if (SetpointPage5 == 0) {
    SetpointPage5 = defaultSetpoint;
  }

  SetpointPage6 = EEPROM.readInt(addressPage6);
  if (SetpointPage6 == 0) {
    SetpointPage6 = defaultSetpoint;
  }

  scale = EEPROM.readDouble(addressScale);
  if (scale == 0) {
    scale = 1;
  }

  gap = EEPROM.readInt(addressGap);
  if (gap == 0) {
    gap = 500;
  }

  threshold = EEPROM.readInt(addressThreshold);
  if (threshold == 0) {
    threshold = 4000;
  }

  arcStabilizeDelay = EEPROM.readInt(addressDelay);
  if (arcStabilizeDelay == 0) {
    arcStabilizeDelay = 150;
  }

  steps_per_mm = EEPROM.readInt(addressSteps);
  if (steps_per_mm == 0) {
    steps_per_mm = 200;
  }

  maxPos = EEPROM.readInt(addressMaxpos);
  if (maxPos == 0) {
    maxPos = 40 * steps_per_mm;
  }

  minPos = EEPROM.readInt(addressMinpos);
  if (minPos == 0) {
    minPos = -(40 * steps_per_mm);
  }

  aggKp = EEPROM.readFloat(addressAP);
  if (aggKp == 0) {
    aggKp = 0.175;
  }

  aggKi = EEPROM.readFloat(addressAI);
  if (aggKi == 0) {
    aggKi = 0.1;
  }

  aggKd = EEPROM.readFloat(addressAD);
  if (aggKd == 0) {
    aggKd = 0.1;
  }

  Kp = EEPROM.readFloat(addressCP);
  if (Kp == 0) {
    Kp = 0.075;
  }

  Ki = EEPROM.readFloat(addressCI);
  if (Ki == 0) {
    Ki = 0.01;
  }

  Kd = EEPROM.readFloat(addressCD);
  if (Kd == 0) {
    Kd = 0.01;
  }

  CalibrationOffset = EEPROM.readInt(addressCalibrate);
  if (CalibrationOffset == 0) {
    CalibrationOffset = 0;
  }

  // Wait for Nextion Screen to bootup
  delay(2500);
  THCNex.writeNum("CustomSetPoint.val", SetpointPage1);
  THCNex.writeNum("CustomSetPoint.val", SetpointPage1); //Make sure it set
  THCNex.writeNum("CustomSetPoint.val", SetpointPage1); //One more time
  SetPoint = SetpointPage1;

  //Setup Stepper Driver
  stepper.setMaxSpeed(1500000); //thru experimentation I found these values to work... Change for your setup.
  stepper.setAcceleration(200000);
  //Enable MKS Driver Board
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);

}


// the loop function runs over and over again forever
void loop()
{
  THCNex.NextionListen();
  Input = map(analogRead(PLASMA_INPUT_PIN), 0, 1023, 0, 25000) + CalibrationOffset; //reads plasma arc voltage and convert to millivolt
  process(); //This is the main method of the application it calulates position and move steps if Input Voltage is over threshold.
  if (CurrentPageNumber <= 6 || CurrentPageNumber == 11) {
    report();
  }
}

void process() //Calulates position and move steps
{
  oldDelay = micros();
  while (Input > (threshold + CalibrationOffset)) //Only move if cutting by checking for voltage above a threshold level
  {
    if (micros() - oldDelay >= arcStabilizeDelay) //wait for arc to stabilize tipically 100-300ms
    {
      Input = map(analogRead(PLASMA_INPUT_PIN), 0, 1023, 0, 25000) + CalibrationOffset; //get new plasma arc voltage and convert to millivolts

      currentGap = abs(SetPoint - Input); //distance away from setpoint
      if (currentGap < gap) {
        THCPID.setCoefficients(Kp, Ki, Kd, Hz); //we're close to setpoint, use conservative tuning parameters
      }
      else {
        THCPID.setCoefficients(aggKp, aggKi, aggKd, Hz); //we're far from setpoint, use aggressive tuning parameters
      }

      if (SetPoint > Input)
      {
        targetInput = Input - SetPoint + SetPoint;
        output = THCPID.step(SetPoint, targetInput);
        pos = pos + output;
      }
      else
      {
        targetInput = SetPoint - Input + SetPoint;
        output = THCPID.step(SetPoint, targetInput);
        pos = pos - output;
      }

      //Validate move is within range
      if (pos >= maxPos) {
        pos = maxPos;
      }
      if (pos <= minPos) {
        pos = minPos;
      }

      //do move
      stepper.moveTo(pos);
      while (stepper.distanceToGo() != 0) {
        stepper.run();
      }

      report(); //report plasma voltage and position
      //format();
    }
  }
  //after cut reset height
  pos = 0;
  //do move
  stepper.moveTo(pos);
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
}

void trigger0() //Set last page used on startup loaded event
{
  //not used because bug with Nextion screen not updating screen loaded event.
  //THCNex.writeNum("dp", SavedPage);
}

void trigger1() //read customsetpoint on page loaded event
{
  CurrentPageNumber = THCNex.readNumber("dp");
  SetPoint = THCNex.readNumber("CustomSetPoint.val");
  if (CurrentPageNumber != 777777 && SetPoint != 777777)
  {
    switch (CurrentPageNumber) {
      case 1:
        SetPoint = SetpointPage1; //write a few times to make sure... nextion screen has a nasty habbat of ignoring update commands on boot.
        THCNex.writeNum("CustomSetPoint.val", SetpointPage1);
        THCNex.writeNum("CustomSlide.val", SetpointPage1);
        THCNex.writeNum("CustomSetPoint.val", SetpointPage1);
        THCNex.writeNum("CustomSlide.val", SetpointPage1);
        THCNex.writeNum("CustomSetPoint.val", SetpointPage1);
        THCNex.writeNum("CustomSlide.val", SetpointPage1);
        break;
      case 2:
        SetPoint = SetpointPage2;
        THCNex.writeNum("CustomSetPoint.val", SetpointPage2);
        THCNex.writeNum("CustomSlide.val", SetpointPage2);
        break;
      case 3:
        SetPoint = SetpointPage3;
        THCNex.writeNum("CustomSetPoint.val", SetpointPage3);
        THCNex.writeNum("CustomSlide.val", SetpointPage3);
        break;
      case 4:
        SetPoint = SetpointPage4;
        THCNex.writeNum("CustomSetPoint.val", SetpointPage4);
        THCNex.writeNum("CustomSlide.val", SetpointPage4);
        break;
      case 5:
        SetPoint = SetpointPage5;
        THCNex.writeNum("CustomSetPoint.val", SetpointPage5);
        THCNex.writeNum("CustomSlide.val", SetpointPage5);
        break;
      case 6:
        SetPoint = SetpointPage6;
        THCNex.writeNum("CustomSetPoint.val", SetpointPage6);
        THCNex.writeNum("CustomSlide.val", SetpointPage6);
        break;
      default:
        break;
    }
  }
}
void trigger2() //Save customsetpoints on end touch event
{
  CurrentPageNumber = THCNex.readNumber("dp");
  SetPoint = THCNex.readNumber("CustomSetPoint.val");
  if (CurrentPageNumber != 777777 && SetPoint != 777777)
  {
    switch (CurrentPageNumber) {
      case 1:
        SetpointPage1 = SetPoint;
        EEPROM.writeInt(addressPage1, SetpointPage1);
        break;
      case 2:
        SetpointPage2 = SetPoint;
        EEPROM.writeInt(addressPage2, SetpointPage2);
        break;
      case 3:
        SetpointPage3 = SetPoint;
        EEPROM.writeInt(addressPage3, SetpointPage3);
        break;
      case 4:
        SetpointPage4 = SetPoint;
        EEPROM.writeInt(addressPage4, SetpointPage4);
        break;
      case 5:
        SetpointPage5 = SetPoint;
        EEPROM.writeInt(addressPage5, SetpointPage5);
        break;
      case 6:
        SetpointPage6 = SetPoint;
        EEPROM.writeInt(addressPage6, SetpointPage6);
        break;
      default:
        break;
    }
  }
}
void trigger3() //Move motor up
{
  pos = pos + (scale * steps_per_mm);
  stepper.moveTo(pos);
  while (stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  THCNex.writeNum("x2.val", (int)(pos / 2));
}
void trigger4() //Move motor down
{
  pos = pos - (scale * steps_per_mm);
  stepper.moveTo(pos);
  while (stepper.distanceToGo() != 0)
  {
    stepper.run();
  }
  THCNex.writeNum("x2.val", (int)(pos / 2));
}
void trigger5() //Increase allowable down movement range
{
  minPos = minPos + (scale * steps_per_mm);
  THCNex.writeNum("x1.val", (int)(minPos / 2));
  EEPROM.writeInt(addressMinpos, minPos);
}
void trigger6() //Decrease allowable up movement range
{ minPos = minPos - (scale * steps_per_mm);
  THCNex.writeNum("x1.val", (int)(minPos / 2));
  EEPROM.writeInt(addressMinpos, minPos);

}
void trigger7() //Increase allowable up movement range
{
  maxPos = maxPos + (scale * steps_per_mm);
  THCNex.writeNum("x0.val", (int)(maxPos / 2));
  EEPROM.writeInt(addressMaxpos, maxPos);

}
void trigger8() //Decrease allowable down movement range
{
  maxPos = maxPos - (scale * steps_per_mm);
  THCNex.writeNum("x0.val", (int)(maxPos / 2));
  EEPROM.writeInt(addressMaxpos, maxPos);
}

void trigger9() //Increase voltage gap between aggressive and normal targeting
{
  gap = gap + (scale * 100);
  THCNex.writeNum("x2.val", (int)(gap));
  EEPROM.writeInt(addressGap, gap);
}

void trigger10() //Decrease voltage gap between aggressive and normal targeting
{
  gap = gap - (scale * 100);
  THCNex.writeNum("x2.val", (int)(gap));
  EEPROM.writeInt(addressGap, gap);
}
void trigger11() //Increase voltage reading threshold for calculating movements
{
  threshold = threshold + (scale * 100);
  THCNex.writeNum("x1.val", (int)(threshold));
  EEPROM.writeInt(addressThreshold, threshold);
}
void trigger12() //Decrease voltage reading threshold for calculating movements
{
  threshold = threshold - (scale * 100);
  THCNex.writeNum("x1.val", (int)(threshold));
  EEPROM.writeInt(addressThreshold, threshold);
}
void trigger13() //Increase delay before calculating movements
{
  arcStabilizeDelay = arcStabilizeDelay + (scale * 100);
  THCNex.writeNum("x0.val", (int)(arcStabilizeDelay / 10));
  EEPROM.writeInt(addressDelay, arcStabilizeDelay);
}
void trigger14() //Decrease delay before calculating movements
{
  arcStabilizeDelay = arcStabilizeDelay - (scale * 100);
  THCNex.writeNum("x0.val", (int)(arcStabilizeDelay / 10));
  EEPROM.writeInt(addressDelay, arcStabilizeDelay);
}
void trigger15() //Increase steps per millimeter
{
  steps_per_mm = steps_per_mm + scale;
  THCNex.writeNum("x3.val", (int)(100 * steps_per_mm));
  EEPROM.writeInt(addressSteps, steps_per_mm);
}
void trigger16() //Decrease steps per millimeter
{
  steps_per_mm = steps_per_mm - scale;
  THCNex.writeNum("x3.val", (int)(100 * steps_per_mm));
  EEPROM.writeInt(addressSteps, steps_per_mm);
}
void trigger17() //Increase Aggressive P Parameter
{
  aggKp = aggKp + scale * 0.01;
  THCNex.writeNum("x2.val", (int)(1000 * aggKp));
  EEPROM.writeFloat(addressAP, aggKp);
}
void trigger18() //Decrease Aggressive P Parameter
{
  aggKp = aggKp - scale * 0.01;
  THCNex.writeNum("x2.val", (int)(1000 * aggKp));
  EEPROM.writeFloat(addressAP, aggKp);
}
void trigger19() //Increase Aggressive I Parameter
{
  aggKi = aggKi + scale * 0.01;
  THCNex.writeNum("x1.val", (int)(1000 * aggKi));
  EEPROM.writeFloat(addressAI, aggKi);
}
void trigger20() //Decrease Aggressive I Parameter
{
  aggKi = aggKi - scale * 0.01;
  THCNex.writeNum("x1.val", (int)(1000 * aggKi));
  EEPROM.writeFloat(addressAI, aggKi);
}
void trigger21() //Increase Aggressive D Parameter
{
  aggKd = aggKd + scale * 0.01;
  THCNex.writeNum("x0.val", (int)(1000 * aggKd));
  EEPROM.writeFloat(addressAD, aggKd);
}
void trigger22() //Decrease Aggressive D Parameter
{
  aggKd = aggKd - scale * 0.01;
  THCNex.writeNum("x0.val", (int)(1000 * aggKd));
  EEPROM.writeFloat(addressAD, aggKd);
}
void trigger23() //Increase Conservative P Parameter
{
  Kp = Kp + scale * 0.01;
  THCNex.writeNum("x2.val", (int)(1000 * Kp));
  EEPROM.writeFloat(addressCP, Kp);
}
void trigger24() //Decrease Conservative P Parameter
{
  Kp = Kp - scale * 0.01;
  THCNex.writeNum("x2.val", (int)(1000 * Kp));
  EEPROM.writeFloat(addressCP, Kp);
}
void trigger25() //Increase Conservative I Parameter
{
  Ki = Ki + scale * 0.01;
  THCNex.writeNum("x1.val", (int)(1000 * Ki));
  EEPROM.writeFloat(addressCI, Ki);
}
void trigger26() //Decrease Conservative I Parameter
{
  Ki = Ki - scale * 0.01;
  THCNex.writeNum("x1.val", (int)(1000 * Ki));
  EEPROM.writeFloat(addressCI, Ki);
}
void trigger27() //Increase Conservative D Parameter
{
  Kd = Kd + scale * 0.01;
  THCNex.writeNum("x0.val", (int)(1000 * Kd));
  EEPROM.writeFloat(addressCD, Kd);
}
void trigger28() //Decrease Conservative D Parameter
{
  Kd = Kd - scale * 0.01;
  THCNex.writeNum("x0.val", (int)(1000 * Kd));
  EEPROM.writeFloat(addressCD, Kd);
}
void trigger29() //load movement page settings
{
  if (scale == 0.1)
  {
    THCNex.writeNum("bt0.val", 1);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 1.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 1);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 10.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 1);
  }
  THCNex.writeNum("x2.val", (int)(pos / 2));
  THCNex.writeNum("x0.val", (int)(maxPos / 2));
  THCNex.writeNum("x1.val", (int)(minPos / 2));
}
void trigger30() //Load default page settings
{
  if (scale == 0.1)
  {
    THCNex.writeNum("bt0.val", 1);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 1.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 1);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 10.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 1);
  }
  THCNex.writeNum("x2.val", (int)(gap));
  THCNex.writeNum("x1.val", (int)(threshold));
  THCNex.writeNum("x0.val", (int)(arcStabilizeDelay / 10));
  THCNex.writeNum("x3.val", (int)(100 * steps_per_mm));
}
void trigger31() //Save Scale on end touch event
{
  if (THCNex.readNumber("bt0.val") == 1)
  {
    scale = 0.1;
  }
  if (THCNex.readNumber("bt1.val") == 1)
  {
    scale = 1;
  }
  if (THCNex.readNumber("bt2.val") == 1)
  {
    scale = 10;
  }
}
void trigger32() //Load Aggressive PID settings
{
  if (scale == 0.1)
  {
    THCNex.writeNum("bt0.val", 1);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 1.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 1);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 10.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 1);
  }
  THCNex.writeNum("x2.val", (int)(1000 * aggKp));
  THCNex.writeNum("x1.val", (int)(1000 * aggKi));
  THCNex.writeNum("x0.val", (int)(1000 * aggKd));
}
void trigger33() //Load Conservative PID Settings
{ if (scale == 0.1)
  {
    THCNex.writeNum("bt0.val", 1);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 1.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 1);
    THCNex.writeNum("bt2.val", 0);
  }
  if (scale == 10.0)
  {
    THCNex.writeNum("bt0.val", 0);
    THCNex.writeNum("bt1.val", 0);
    THCNex.writeNum("bt2.val", 1);
  }
  THCNex.writeNum("x2.val", (int)(1000 * Kp));
  THCNex.writeNum("x1.val", (int)(1000 * Ki));
  THCNex.writeNum("x0.val", (int)(1000 * Kd));
}
void trigger34() //Load Calibration Offset
{
  THCNex.writeNum("CustomSetPoint.val", CalibrationOffset);
}
void trigger35() //Save Calibration Offset on end touch event
{
  int cali = THCNex.readNumber("CustomSetPoint.val");
  if (cali != 77777) {
    CalibrationOffset = cali;
    EEPROM.writeInt(addressCalibrate, CalibrationOffset);
  }
}
void report() //report plasma voltage and position
{
  THCNex.writeNum("PV.val", (int)Input);
  THCNex.writeNum("POS.val", (int)(pos / 2));
}
void format() //Set text color
{
  if (pos > 1 && pos < 1000)
  {
    THCNex.writeNum("POS.pco", 4065);
  }
  if (pos < -1 && pos > -1000)
  {
    THCNex.writeNum("POS.pco", 63488);
  }
}
