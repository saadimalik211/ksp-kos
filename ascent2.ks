// Program Title: LaunchToOrbit
// Author: JitteryJet
// Version: V03
// kOS Version: 1.3.2.0
// KSP Version: 1.12.2
// Description:
//  Launch a vessel into a circular orbit.
//
//Notes:
//  Features:
//    - The launch trajectory uses a Gravity Turn maneuver with a zero Angle of Attack (AOA),
//      the trajectory can be tuned with the start turn and pitchover angle parameters.
//    - The vessel can be launched from an airless body, the zero AOA is skipped because
//      it is not necessary.
//    - The vessel can be launched from any location, not just the KSC launchpad.
//    - The final orbit can be inclined.
//    - KSP Maneuver Nodes are not used.
//
//  Out of Fuel:
//    - Out of Fuel is ignored. If there is not enough fuel, results will be
//      unpredictable.
//
//  The North and South Pole Krakens:
//    - If a launch is done near the North or South Poles of a body the results may be unpredictable.
//      This is because the vessel's NORTH vector becomes undefined at the poles (think about it...).
//      The vessel's NORTH vector is used to calculate trajectories eg HEADING function etc.
//
//  Vessel Design:
//    - The vessel is assumed to have enough steering to allow the kOS Steering Manager to
//      maneuver the vessel.
//    - The vessel is assumed to have enough thrust in each stage to allow each maneuver
//      to complete.
//
// Todo:
//    - Fix bug with reference to inclined orbits.
//    - Fix slight drift during ascent esp inclined orbits.
//    - Review thrust calculations for circularisation maneuver.
//    - Investigate issue where throttle down does not cut in
//      when the desired apoapsis is very high. This results in overshoot.
//    - Improve handling of situation when there is not enough time to
//      circularize the orbit before reaching the desired orbit altitude.
//    - Allow other types of gravity turns to be used.
//
// Update History:
//    07/03/2020 V01  - Created.
//    16/08/2020 V02  - Add maneuver steering time as a parameter.
//                    - Change orbit altitude from meters to kilometers.
//                    - Fixed staging issue where an engine "flame out" does
//                      not mean the stage has completely run out of fuel.
//                    - Declare local functions LOCAL to ensure they are
//                      not accidentally called from other scripts. The
//                      default scope for a function is GLOBAL.
//                    - Fixed handling for airless bodies.
//    27/11/2021 V03  - WIP.
//                    - Test on Eve.
//                    - Allow launching straight up ie pitchover of zero.
//                    - Remove "cabin up" roll at launch.
//                    - Remove unnecessary "wait 0".
//                    - Remove unnecessary lock identifers from circularization code.
//                    - Upgraded to MiscFunctions V04.
//                    - Rewrote Multi Function Display (MFD).
//                    - Replaced the PID Loop controlled circularization burn with
//                      a timed circularization burn.
//                    - Test with KSP 1.12.2.
//
// Run parameters declarations.
//	OrbitAltitude				    Sea level altitude of the final circular orbit (m).
//  OrbitInclination        The desired inclination of the final orbit (deg).
//	LaunchDirection			    Direction to launch to NORTH or SOUTH.
//	TurnStartAltitude			  Terrain altitude of the start of the turn (m).
//  TurnPitchoverAngle      Angle of the initial pitchover to start the gravity turn (deg).  
//  SteeringDuration        Time to allow the vessel to steer to the circularization burn
//                          direction.              
//  WarpType		  				  "PHYSICS","RAILS" or "NOWARP".
//	LaunchCountdownPeriod		Period of time before launching (s).
//	SyncLaunch					    Synchronised launch switch. SYNC or NOSYNC.

@lazyglobal off.

parameter OrbitAltkm to 120.
parameter OrbitInclination to 0.
parameter LaunchDirection to "NORTH".
parameter TurnStartAltitude to 500.
parameter TurnPitchoverAngle to 10.
parameter SteeringDuration to 60.
parameter WarpType to "NOWARP".
parameter LaunchCountdownPeriod to 10.
parameter SyncLaunch to "NOSYNC".

// Load in library functions.
runoncepath("MiscFunctions V04").
runoncepath("LaunchToOrbitMFD V01").

// Global variable declarations.
local OrbitAltitude to OrbitAltkm*1000.
local LaunchCountdownCtr to LaunchCountdownPeriod.
local LaunchAltitude to ship:altitude.
local LaunchAzimuth to CalcLaunchAzimuth().
local CircDeltaV to 0.
local CircDeltaVVec to 0.
local CircBurnStart to 0.
local CircEstBurnTime to 0.
local FatalError to false.
local MFDRefreshTriggerActive to true.
local BurnStartTimeSecs to 0.

// Other initialisations.
set ship:control:pilotmainthrottle to 0.
SAS off.
RCS off.
lights on.

// Main program.
clearscreen.
SetMFD().
CheckForErrorsAndWarnings().

if not FatalError
  {
    CreateMFDRefreshTrigger().
    LaunchCountdown().
    Launch().
    Ascend().  
    CoastToCircularization().
    Circularize().
  }

//wait until false.
RCS off.
RemoveLocksAndTriggers().

local function LaunchCountdown
  {
// Count down to the launch.
// Notes:
//    -
// Todo:
//    -
		if SyncLaunch = "SYNC"
			SynchroniseLaunch().
    LaunchToOrbitMFD["DisplayFlightStatus"]("Countdown").
    set BurnStartTimeSecs to time:seconds+LaunchCountdownPeriod.
    from {}
    until LaunchCountdownCtr = 0
    step {set LaunchCountdownCtr to LaunchCountdownCtr - 1.}
    do
      {
        wait 1.
      }
  }

local function Launch
  {
// Launch the vessel.
// Notes:
//    - The vessel will launch straight up with no roll.
// Todo:
//    -
    lock throttle to 1.
    lock steering to lookdirup (ship:up:forevector,ship:facing:topvector).
		if ship:status = "PRELAUNCH"
    	stage.
    LaunchToOrbitMFD["DisplayFlightStatus"]("Launch").
    SetStagingTrigger.
    until (ship:altitude-LaunchAltitude >= TurnStartAltitude)
	    wait 0.
    legs off.
  }

local function Ascend
  {
// Ascend until the vessel's orbit reached the target orbital altitude.
// Notes:
//    - The ascent follows a "gravity turn" trajectory.
//    - When in an atmosphere the vessel is steered to the surface prograde.
//      This results in a very low or zero Angle of Attack (AOA) trajectory
//      which fits the definition of a "zero lift gravity turn".
//    - If there is no atmosphere the vessel is steered to the pitchover angle.
//    - The ascent turn is considered complete when the desired apoapsis is reached and 
//      the vessel clears any atmosphere.
//    - Apoapsis overshoot is controlled by throttle down.
//    - Strictly speaking a PID Loop is not required. But I wanted to use a PID Loop
//      anyway - you be the judge.
//    - The PID Proportional Gain is used to control the throttle down.
//      The PID Integral Gain is used to nudge the throttle down at apoapsis.
//    - I do not understand SteeringManager:angleerror so I do not use it to check for
//      Steering Manager settling.
//
// Todo:
//    - Check the steering used if launched from an airless body - the steering is
//      probably not correct. Use of the HEADING function might cause the steering to
//      drift off the intended orbital plane.
    local AscentCompleted to false.
    local ThrottleLockedToPID to false.

// KP PID gain - Use 1/KP meters throttleback eg 1/1000 for throttle down during last 1 km of apoapsis increase.
    local KP to 1/1000.
// KI PID gain - Keep very small. It is required to nudge the throttle to ensure apoapsis is reached and the burn loop completes.
    local KI to 1E-6.
    local KD to 0. 
    local MinThrottle is 0.
    local MaxThrottle is 1.
    local ThrottlePID to PIDLoop(KP,KI,KD,MinThrottle,MaxThrottle).
    set ThrottlePID:setpoint to OrbitAltitude.

    lock throttle to ThrottlePID:update(time:seconds,ship:orbit:apoapsis).
    set ThrottleLockedToPID to true.

    if TurnPitchoverAngle <> 0
      {    
        LaunchToOrbitMFD["DisplayFlightStatus"]("Pitchover").
        lock steering to lookdirup(heading(LaunchAzimuth,90-TurnPitchoverAngle):forevector,ship:facing:topvector).
// Advance to next physics tick to ensure steering manager output is updated.
        wait 0.
        if ship:body:atm:exists
          {
// Wait for the initial pitchover to settle.
            until vang(ship:facing:forevector,steeringManager:target:forevector) < 1
              wait 0.
// Wait for the surface prograde vector to align with the vessel.
            LaunchToOrbitMFD["DisplayFlightStatus"]("AOA Settle").
            until vang(ship:facing:forevector,ship:srfprograde:forevector) < 2
              wait 0.
            lock steering to lookDirUp(ship:srfprograde:forevector,ship:facing:topvector).
          }
      }
    LaunchToOrbitMFD["DisplayFlightStatus"]("Ascent").
    until AscentCompleted
      {
        if ship:orbit:apoapsis < OrbitAltitude
          {
// Allow for throttle having to be engaged again because the apoapsis dropped due to friction on the vessel
// in an atmosphere or the work of aliens.
            if not ThrottleLockedToPID
              {
                lock throttle to ThrottlePID:update(time:seconds,ship:orbit:apoapsis).
                set ThrottleLockedToPID to true.
              }
          }
        else
          {
            lock throttle to 0.
            set ThrottleLockedToPID to false.
            if ship:altitude >= ship:body:atm:height
                set Ascentcompleted to true.
          }
        wait 0.
      }
    set BurnStartTimeSecs to 0.
  }

local function CoastToCircularization
  {
// Coast to the beginning of the circularization maneuver.
// Notes:
//    - The vessel is steered into the correct attitude for the circularization maneuver
//      and held there after ascend completion. This is at the expense of resources such as
//      monopropellant required to set and hold the attitude. The rationale for doing this is to
//      get the steering done before the circularization maneuver point is reached.
//    -
// Todo:
//		-
    local ManeuverPointUT to time+eta:apoapsis.
    LaunchToOrbitMFD["DisplayFlightStatus"]("Coast to circ").
    set CircDeltaV to CircularizationDeltaV().
    set CircEstBurnTime to DeltaVBurnTimeIdeal(CircDeltaV).
    LaunchToOrbitMFD["DisplayManeuver"]
      (
        CircEstBurnTime,
        CircDeltaV
      ).
    set CircBurnStart to ManeuverPointUT:seconds-CircEstBurnTime/2.
    set BurnStartTimeSecs to CircBurnStart.
    local BurnVec to velocityat(ship,ManeuverpointUT):orbit.
    local SteeringDir to lookdirup(BurnVec,ship:facing:topvector).
    lock steering to SteeringDir.
    LaunchToOrbitMFD["DisplayFlightStatus"]("Steering wait").
    wait min(SteeringDuration,CircBurnStart-time:seconds-SteeringDuration).
// Wait until the start of the circularization maneuver is reached.
    if time:seconds < CircBurnStart
      {
        if WarpType <> "NOWARP"
          {
            wait 1. // Small wait to allow acceleration value to settle.
            WarpToTime(CircBurnStart-SteeringDuration,WarpType).
          }
      }
    wait until time:seconds >= CircBurnStart.
  }

local function Circularize
  {
// Circularize the orbit.
// Notes:
//		-
//    -
// Todo:
//		- 
    local ManeuverPointUT to time+eta:apoapsis.
		set CircDeltaVVec to velocityat(ship,ManeuverPointUT):orbit*CircDeltaV.
    LaunchToOrbitMFD["DisplayFlightStatus"]("Circularizatn").
    OrbitalBurn(CircDeltavVec,CircEstBurnTime).
    LaunchToOrbitMFD["DisplayFlightStatus"]("Circ finished").
	}

local function CalcLaunchAzimuth
  {
// Return the Launch Azimuth which is the angle from north required to
// launch into a specified inclined orbit from a launch site.
// Allow for the rotation of the SOI body at the launch site.
// Notes:
//  01. Calculation variables:
//      Orbit Inclination. (0-180).
//        Desired orbital inclination.
//      Launch Type. Values ("NORTH","SOUTH").
//        Launch to the north or launch to the south. This shows which of
//        the two solutions for a given orbital inclination is required.
//      Inertial azimuth. Values (-90 to 90).
//        First approximation. Does not include compensation for body rotation.
//      Rotational azimuth. Values (-90 to 90).
//        Inertial azimuth plus compensation for body rotation.
//      Launch azimuth. Values (0-360).
//        The compass heading to launch to.
//      Equatorial velocity.
//        Rotational velocity of the body's equator at sea level.
//      Orbital velocity.
//        Orbital velocity at the target orbit.
//  02. The plane of the orbit cannot be closer to the equator than the
//      latitude of the launch site. In this case set the inclination to
//      the same as the launch site ie launch due east or due west depending
//      on prograde or retrograde.
//  03. The formula to calculate inertial azimuth usually produces two
//      solutions that results in the same orbital inclination. These solutions
//      are "North" launches and "South" launches - which one is wanted has to
//      be specified.
    local cosLat to cos(ship:latitude).
    local cosOrbitInc to cos(OrbitInclination).
    local InAz to 0.
    local RotAz to 0.
    local ObtV to 0.
    local EqV to 0.
    local LaunchAz to 0.

    if abs(cosOrbitInc) <= abs(cosLat)
      set InAz to arcsin(cosOrbitInc/cosLat).
    else
      if OrbitInclination <= 90
        set InAz to 90.  // Launch due east.
      else
        set InAz to -90. // Launch due west.

    set ObtV to sqrt (ship:body:mu/(ship:body:radius+OrbitAltitude)).
    set EqV to 2*constant:pi*ship:body:radius/ship:body:rotationperiod.
    set RotAz to arctan((ObtV*sin(InAz)-EqV*cosLat)/(ObtV*cos(InAz))).

    //set RotAz to InAz.

// Convert the azimuth calculated to a compass heading azimuth.
    if LaunchDirection = "NORTH"
      if RotAz >= 0
        set LaunchAz to RotAz.
      else
        set LaunchAz to 360 + RotAz.
    else
    if LaunchDirection = "SOUTH"
      set LaunchAz to 180 - RotAz.

    return LaunchAz.
  }

local function SetStagingTrigger
  {
// Stage automatically the stage can no longer produce thrust.
// Notes:
//		-
// Todo:
//		- Test with sepratrons - they count as solid fuel.
    when
      ship:maxthrust = 0
//      or (stage:liquidfuel = 0 and stage:solidfuel = 0)
      or (stage:liquidfuel = 0)
    then
      {
        stage.
        until stage:ready {wait 0.}
        if stage:number > 0
          return true.
        else
          return false.
      }
  }

local function CircularizationDeltaV
  {
// Calculate the change in velocity required to raise the periapsis to the same
// altitude as the apoapsis.
// Refer to the "Vis-viva" equation.
    local dVold to sqrt(ship:body:mu * (2/(ship:body:radius+ship:obt:apoapsis) -
      1/ship:obt:semimajoraxis)).
    local dVnew to sqrt(ship:body:mu * (1/(ship:body:radius+ship:obt:apoapsis))).
    return (dVnew - dVold).
  }

local function DeltaVBurnTimeIdeal
  {
// Estimate the burn time for the specified deltaV based on the vessel
// characteristics. 
// Notes:
//    - The equation allows for changes in mass as fuel is burnt.
//      Refer to the "Ideal Rocket Equation".
//    - The estimate assumes that thrust and ISP remain constant. These
//      assumptions do not allow for any staging etc that can occur during a
//      burn.
    parameter dV.

    local minitial is 0.
    local mfinal is 0.
    local ISP is 0.
    local g0 is 9.82.
    local mpropellent is 0.
    local mdot is 0.
    local thrust is 0.
    local BurnTime is 0.

    set minitial to ship:mass.
    set thrust to ship:availablethrust.
    set ISP to CurrentISP().

    set mfinal to minitial*constant:e^(-dV/(ISP*g0)).
    set mpropellent to minitial-mfinal.
    set mdot to thrust/(ISP*g0).
    set BurnTime to mpropellent/mdot.

    return BurnTime.
  }

local function CurrentISP
  {
// Calculate the current ISP of the vessel.
// Notes:
//    -
// To do:
//    - Does not allow for different engine types on the stage - the contribution
//      to the ISP is probably dependent on the thrust of the engine as well.
    local ISP is 0.
    local englist is 0.
    list engines in englist.
    for eng in englist
      {
        if eng:stage = stage:number
          set ISP to ISP + eng:isp.
      }
    return ISP.
  }

local function SynchroniseLaunch
  {
// Synchronise the launch to the start of the next "period".
// Notes:
//		- The purpose of a synchronised launch is to allow more than one vessel
//			to be launched at the same time for races, synchronised flying etc.
//		- Is it possible a whole second could be missed? Unlikely??
// ToDo:
//		- This sychronisation method is a bit rough. It can be improved.
		local period to 180.

    LaunchToOrbitMFD["DisplayFlightStatus"]("Sync launch").
		set Launchcountdownctr to
			LaunchCountdownCtr +
			period-mod(floor(time:seconds),period)-1.

	}

local function SetMFD
  {
// Set the Multi-function Display.
// Notes:
//    -
// Todo:
//    -
    clearScreen.
    set terminal:width to 52.
    set terminal:height to 25.
    LaunchToOrbitMFD["DisplayLabels"]
      (
        ship:name,
        TurnStartAltitude,
        TurnPitchoverAngle,
        OrbitAltitude,
        OrbitInclination,
        LaunchDirection,
        LaunchAzimuth
      ).
  }

local function CreateMFDRefreshTrigger
  {
// Create a trigger to refresh the the Multi-function Display periodically.
// Notes:
//    - The pitch angle is the same as the navball pitch angle.
// Todo:
//		-
//    -
    local RefreshInterval to 0.1.
    local NextMFDRefreshTime to time:seconds.
    lock pitch to 90-vang(ship:up:forevector,ship:facing:forevector).
    local StageNum to 0.
    when NextMFDRefreshTime < time:seconds
    then
      {
        if ship:status = "PRELAUNCH"
          set StageNum to stage:number-1.
        else
          set StageNum to stage:number.
        LaunchToOrbitMFD["DisplayRefresh"]
         (
          StageNum, 
          pitch,
          ship:apoapsis,
          ship:periapsis,
          ship:orbit:eccentricity,
          BurnStartTimeSecs,
          time():seconds
         ).
        set NextMFDRefreshTime to NextMFDRefreshTime+RefreshInterval.
        return MFDRefreshTriggerActive.
      }
  }

local function CheckForErrorsAndWarnings
  {
// Check for errors and warnings.
// Notes:
//    -
// Todo:
//    -
    if ship:status <> "PRELAUNCH"
      and ship:status <> "LANDED"
      {
        LaunchToOrbitMFD["DisplayError"]("Vessel is in flight or has splashed down").
        set FatalError to true.
      } 
  }

local function RemoveLocksAndTriggers
{
// Remove locks and triggers.
// Notes:
//    - Guarantee unneeded locks and triggers are removed before
//      any following script is run. THROTTLE, STEERING and
//      triggers are global and will keep processing
//      until control is returned back to the terminal program -
//      this is relevant if this script is ran using
//      RUNPATH from another script before exiting to the
//      terminal program.
//    -
// Todo:
//    -
  set MFDRefreshTriggerActive to false.
  unlock throttle.
  unlock steering.
  wait 0.
}