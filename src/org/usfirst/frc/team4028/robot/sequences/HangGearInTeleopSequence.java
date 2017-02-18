package org.usfirst.frc.team4028.robot.sequences;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;

import edu.wpi.first.wpilibj.DriverStation;

// this class contains the sequence code to hang a gear
//	Assumptions:
//		Initial State
//			Robot is holding gear abotu x degree  tilt angle
//			Spike is thru the gear
//			Gear is almost all the way forward on the spike
//
//		Sequence
//			3 way simultaneous move
//				a) gear Outfeed
//				b) gear tilt down
//				c) drive in reverse
//
//		Final State
//			Robot has backed off the spike
//			gear infeed is stopped
//			gear tile is at x degrees
public class HangGearInTeleopSequence  
{
	// define class level variables for Robot subsystems
	private GearHandler _gearHandler;
	private Chassis _chassis;
	
	// define class level working variables
	private long _seqStartedTimeStamp;
	private boolean _isStillRunning;
	
	// define class level constants
	public static final long MAX_TIME_BEFORE_ABORT_IN_MSEC = 2000; 
	public static final double DRIVE_BACKWARDS_SPEED = 00.50;
	public static final double GEAR_OUTFEED_SPEED = -1.0;
	public static final double GEAR_TILT_SPEED = 1.0;
	public static final int MSEC_FIRST_CHANGE = 300;
	public static final int MSEC_SECOND_CHANGE = 350;
	public static final int MSEC_THIRD_CHANGE = 750;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public HangGearInTeleopSequence(GearHandler gearHandler, Chassis chassis)
	{
		// these are the subsystems that this sequence needs to control
		_gearHandler = gearHandler;
		_chassis = chassis;
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	
	// execute any logic to initialize this object before ExecuteSequenceRentrant is called
	public void Initialize()
	{
		if(!_gearHandler.IsGearInScoringPosition())
		{
			DriverStation.reportError("Cannot start sequence: gear not in scoring position.", false);
		}
		else
		{
		_seqStartedTimeStamp = System.currentTimeMillis();
		_isStillRunning = true;
		
		DriverStation.reportWarning("===== Entering HangGearInTeleopSequence =====", false);
		}
	}
		
	
	// execute the sequence, return = true indicates sequence is still running
	public boolean ExecuteSequenceRentrant()
	{	
		// safety valve since in this mode we take away operator control temporarily
		long elapsedTimeInMSec = System.currentTimeMillis() - _seqStartedTimeStamp;
		
		if(elapsedTimeInMSec <MSEC_FIRST_CHANGE)   //Intial State of Gear
		{
			_gearHandler.MoveTiltAxisVBus(GEAR_TILT_SPEED);    //Sets gear tilt speed and outfeed speed
			_gearHandler.SpinInfeedWheelsVBus(GEAR_OUTFEED_SPEED);
			_chassis.Drive(DRIVE_BACKWARDS_SPEED, 0);
		}
		else if(elapsedTimeInMSec > MSEC_FIRST_CHANGE && elapsedTimeInMSec < MSEC_SECOND_CHANGE)  //second stage of gear Sequence
		{
			_gearHandler.MoveTiltAxisVBus(GEAR_TILT_SPEED);     //sets tilt axis speed, outfeed speed, and drive speed
			_gearHandler.SpinInfeedWheelsVBus(GEAR_OUTFEED_SPEED);
			_chassis.Drive(DRIVE_BACKWARDS_SPEED, 0);
		}
		else if(elapsedTimeInMSec > MSEC_SECOND_CHANGE && elapsedTimeInMSec < MSEC_THIRD_CHANGE) // third state of gear Sequence
		{
			_gearHandler.MoveTiltAxisVBus(GEAR_TILT_SPEED);		//sets drive speed, starts zeroing of axis
			_gearHandler.SpinInfeedWheelsVBus(0);
			_chassis.Drive(DRIVE_BACKWARDS_SPEED, 0);
			//_gearHandler.ZeroGearTiltAxisReentrant();
		}
		else if(elapsedTimeInMSec > MSEC_THIRD_CHANGE && elapsedTimeInMSec < MAX_TIME_BEFORE_ABORT_IN_MSEC)	//final state of gear sequence
		{
			_gearHandler.ZeroGearTiltAxisReentrant();		//zeros tilt
			_gearHandler.SpinInfeedWheelsVBus(0);
			_chassis.Drive(0, 0);
		}
		else if(elapsedTimeInMSec >= MAX_TIME_BEFORE_ABORT_IN_MSEC)  //timeout sequence
		{
			DriverStation.reportWarning("=!=!= HangGearInTeleopSequence Timeout ABORT =!=!=", false);
			return false;
		}
		
		//TODO: Implement (probably) elapsed time based functionality
		//			set _isStillRunning to false when complete!
		
		return _isStillRunning;
	}

	//============================================================================================
	// Properties follow
	//============================================================================================
}
