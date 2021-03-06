package org.usfirst.frc.team4028.robot.sequences;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;

import edu.wpi.first.wpilibj.DriverStation;

// this class contains the sequence code to hang a gear
//	Assumptions:
//		Initial State
//			Robot is holding gear about x degree  tilt angle
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
//			gear tilt is at x degrees
//
//------------------------------------------------------
//	Rev		By		 	D/T			Desc
//	===		========	===========	=================================
//	0		Nick	 	17.Feb.2017	Initial Version
//------------------------------------------------------
//
//=====> For Changes see Nick Donahue (javadotmakeitwork)
public class HangGearInTeleopSequence  
{
	// define class level variables for Robot subsystems
	private GearHandler _gearHandler;
	private Chassis _chassis;
	
	// define class level working variables
	private long _seqStartedTimeStamp;
	private boolean _isStillRunning;
	
	// define class level constants
	private static final long MAX_TIME_BEFORE_ABORT_IN_MSEC = 2000; 
	private static final double DRIVE_BACKWARDS_SPEED = 00.50;
	private static final double GEAR_OUTFEED_SPEED = -1.0;
	private static final double GEAR_TILT_SPEED = 00.75;
	private static final int MSEC_FIRST_CHANGE = 300;
	private static final int MSEC_SECOND_CHANGE = 750;
	
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
		

		if(elapsedTimeInMSec < MSEC_FIRST_CHANGE)   //Initial State of Gear

		{
			_gearHandler.MoveTiltAxisVBus(GEAR_TILT_SPEED);    //Sets gear tilt speed and outfeed speed, drives backwards
			_gearHandler.SpinInfeedWheelsVBus(GEAR_OUTFEED_SPEED);
			_chassis.ArcadeDrive(DRIVE_BACKWARDS_SPEED, 0);			// 0 = no turn
			_isStillRunning = true;
		}
		else if(elapsedTimeInMSec > MSEC_FIRST_CHANGE && elapsedTimeInMSec <= MSEC_SECOND_CHANGE) // third state of gear Sequence
		{
			_gearHandler.MoveTiltAxisVBus(0);		//sets drive speed, starts zeroing of axis
			_gearHandler.SpinInfeedWheelsVBus(0);
			_chassis.ArcadeDrive(DRIVE_BACKWARDS_SPEED, 0);
			_isStillRunning = true;
		}
		else if(elapsedTimeInMSec > MSEC_SECOND_CHANGE && elapsedTimeInMSec < MAX_TIME_BEFORE_ABORT_IN_MSEC)	//final state of gear sequence
		{
			_gearHandler.MoveGearToHomePosition();
			_gearHandler.SpinInfeedWheelsVBus(0);
			_chassis.ArcadeDrive(0, 0);
			_isStillRunning = false;			//ends sequence

		}
		else if(elapsedTimeInMSec >= MAX_TIME_BEFORE_ABORT_IN_MSEC)  //timeout in order to end sequence
		{
			DriverStation.reportWarning("=!=!= HangGearInTeleopSequence Timeout ABORT =!=!=", false);


			_isStillRunning = false;
		}

		return _isStillRunning;
	}

	//============================================================================================
	// Properties follow
	//============================================================================================
}
