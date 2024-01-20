// Name: LaunchToOrbitMFD
// Author: JitteryJet
// Version: V01
// kOS Version: 1.3.2.0
// KSP Version: 1.11.2
// Description:
//    Multi-function Display (MFD) for the LaunchToOrbit program.
//
//Notes:
//    - Coded using Delegates.
//    - Current vessel only ie the vessel running the calling program.
//    - It should be possible to call this script from programs other than StarshipSimulator,
//      but it was designed with StarshipSimulator in mind.
//    - The clearscreen and a big enough terminal size are set in the caller, I didn't want to
//      make any assumptions about how the caller might want the terminal set.
//    - The screen can be refreshed as little or as often as possible.
//      The refresh can be done by a trigger, but keep it mind it will use up some of the
//      physics tick.
//
// Todo:
//    - Test different refresh rates to see which one works the best.
//
// Update History:
//    04/05/2021 V01  - WIP
//                    - Created.
//
// MFD Label Abbreviations:
//  Function      MFD Function Name.
//  Vessel Name   Name of this vessel.
//	TurnS	        Gravity Turn Start Altitude (km).
//  TurnP         Gravity Turn Initial Pitch (deg).
//	Ap            Orbit Apoapsis (km).
//	Pe	          Orbit Periapsis (km).
//  Ecc           Orbit Eccentricity.
//  Ptch          Pitch (deg).
//	Stat	        Flight Status.
//	TAlt	        Target Orbital Altitude (km).
//  TInc          Target Orbital Inclination (deg).
//	Stge	        Vessel Stage number (0 final stage).
//	LDir	        Launch Direction (North or South).
//	LAzi	        Launch Azimuth (deg).
//	BnIn	        Maneuver Burn In Time (s).
//  BnDu	        Maneuver Burn Duration (s).
//	BnDv	        Maneuver Burn Delta-v (m/s).
//
@lazyglobal off.
// Expose the delegates.
global LaunchToOrbitMFD to lexicon
  (
    "DisplayLabels",DisplayLabels@,
    "DisplayRefresh",DisplayRefresh@,
    "DisplayDiagnostic",DisplayDiagnostic@,
    "DisplayManeuver",DisplayManeuver@,
    "DisplayFlightStatus",DisplayFlightStatus@,
    "DisplayError",DisplayError@
  ).

// Variables to keep track of datum line numbers.
local StageLine to 0.
local PitchLine to 0.
local TurnPLine to 0.
local TurnSLine to 0.
local LDirLine to 0.
local LAziLine to 0.
local TAltLine to 0.
local TIncLine to 0.
local ApLine to 0.
local PeLine to 0.
local EccLine to 0.
local Nameline to 0.
local StatLine to 0.
local Diag1Line to 0.
local Diag2Line to 0.
local BnInLine to 0.
local BnDuLine to 0.
local BnDvLine to 0.
local Error1Line to 0.
local Col1Col to 7.
local Col2Col to 35.
local ColSize to 15.
local LineSize to 50.

local function DisplayLabels
  {
// Display the labels, headings and any data that does not change.
// Calculate the line numbers for each datum.
// Notes:
//    - The two blank lines at the top of the screen allow for the
//      "Program ended" message line and the following cursor line.
//
    parameter VesselName.
    parameter TurnS.
    parameter TurnP.
    parameter TAlt.
    parameter TInc.
    parameter LDir.
    parameter LAzi.

    local line to 0.

//         -123456789-123456789-123456789-123456789-123456789
//         XXXXXX XXXXXXXXXXXXXXX      XXXXXX XXXXXXXXXXXXXXX
    print "                                                  " at (0,line).
    set line to line+1.
    print "                                                  " at (0,line).
    set line to line+1.
    print "Function: Launch To Orbit                         " at (0,line).
    set line to line+1.
    print "Vessel Name:                                      " at (0,line).
    set NameLine to line. set line to line+1.
    print "------VESSEL----------      -------AUTOPILOT------" at (0,line).
    set line to line+1.
    print "Stage:                      TurnS:                " at (0,line).
    set StageLine to line. set TurnSLine to line. set line to line+1.
    print "Pitch:                      TurnP:                " at (0,line).
    set PitchLine to line. set TurnPLine to line. set line to line+1.
    print "--------ORBIT---------      Stat:                 " at (0,line).
    set StatLine to line. set line to line+1.
    print "Ap:                         Alt:                  " at (0,line).
    set ApLine to line. set TAltLine to line. set line to line+1.
    print "Pe:                         Inc:                  " at (0,line).
    set PELine to line. set TIncLine to line. set line to line+1.
    print "Ecc:                        LDir:                 " at (0,line).
    set EccLine to line. set LDirLine to line. set line to line+1.
    print "                            LAzi:                 " at (0,line).
    set LAziLine to line. set line to line+1.
    print "----ESTIMATED BURN----                            " at (0,line).
    set line to line+1.
    print "BnIn:                                             " at (0,line).
    set BnInLine to line. set line to line+1.
    print "BnDu:                                             " at (0,line).
    set BnDuLine to line. set line to line+1.
    print "BnDv:                                             " at (0,line).
    Set BnDvLine to line. set line to line+1.
    print "-------ERROR MSG------                            " at (0,line).
    set line to line+1.
    print "                                                  " at (0,line).
    set Error1Line to line. set line to line+1.
    print "-----DIAGNOSTICS------                            " at (0,line).
    set line to line+1.
    print "                                                  " at (0,line).
    set Diag1Line to line. set line to line+1.
    print "                                                  " at (0,line).
    set Diag2Line to line. set line to line+1.

    print VesselName at (13,Nameline).
    print MFDVal(round(TAlt/1000,3) + " km") at (Col2Col,TAltLine).
    print MFDVal(round(TInc,3) + char(176)) at (Col2Col,TIncLine).
    print MFDVal(round(TurnP,3) + char(176)) at (Col2Col,TurnPLine).
    print MFDVal(round(TurnS/1000, 3) + " km") at (Col2Col,TurnSLine).
    print MFDVal(LDir) at (Col2Col,LDirLine).
    print MFDVal(round(LAzi,3) + char(176)) at (Col2Col,LAziLine).
      
  }

local function DisplayRefresh
    {
// Refresh the changing MFD data on the display.
// Notes:
//    -
      parameter StageNum.
      parameter pitch.
      parameter Ap.
      parameter Pe.
      parameter Ecc.
      parameter BurnStartTimeSecs.
      parameter UTSecs.

      print MFDVal(StageNum) at (Col1Col,StageLine).
      print MFDVal(round(pitch,3) + char(176)) at (Col1Col,PitchLine).
      print MFDVal(round(Ap/1000,3)+" km") at (Col1Col,ApLine). 
      print MFDVal(round(Pe/1000,3)+" km") at (Col1Col,PeLine).
      print MFDVal(round(Ecc,5)) at (Col1Col,EccLine).

    if BurnStartTimeSecs = 0
      print "":padright(Colsize) at (Col1Col,BnInLine).
    else
      {
        if BurnStartTimeSecs >= UTSecs
          print MFDVal("T-"+round(abs(UTSecs-BurnStartTimeSecs),1)+" s") at (Col1Col,BnInLine).
        else
          print MFDVal("T+"+round(UTSecs-BurnStartTimeSecs,1)+" s") at (Col1Col,BnInLine).
      }
    }

local function DisplayManeuver
    {
// Update the manuever data.

      parameter BnDu.
      parameter BnDv.

      print MFDVal(round(BnDu,1)+" s") at (7,BnDuLine).
      print MFDVal(round(BnDv,1)+" m/s") at (7,BnDvLine).
    }

local function DisplayFlightStatus
    {
// Update the Flight Status data.

      parameter stat.

      print stat:tostring:padright(ColSize) at (Col2Col,StatLine).
    }

local function DisplayError
    {
// Update the error info lines.
      parameter ErrorLine1 to "".

      print ErrorLine1:tostring:padright(LineSize) at (0,Error1Line).
    }

local function DisplayDiagnostic
    {
// Update the diagnostic info lines.
      parameter DiagLine1 to "".
      parameter DiagLine2 to "".

      print DiagLine1:tostring:padright(LineSize) at (0,Diag1Line).
      print DiagLine2:tostring:padright(LineSize) at (0,Diag2Line).
    }

local function MFDVal
    {
// Format the Multi-function Display value.
// Pad the value from the left with spaces to right-align it.
// If the value is too large, truncate it from the left.
      parameter Val is "".

      local FmtVal is "".

      if Val:istype("Scalar")
        set Val to Val:tostring().

      if Val:length >= ColSize
        set FmtVal to Val:padright(ColSize).
      else
        set FmtVal to Val:padleft(ColSize).

      return FmtVal.
    }
//}