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
	public static final long MAX_TIME_BEFORE_ABORT_IN_MSEC = 5000; 
	public static final double DRIVE_BACKWARDS_SPEED = 0.2;
	public static final double GEAR_OUTFEED_SPEED = 0.5;
	
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
		
		if(elapsedTimeInMSec >= MAX_TIME_BEFORE_ABORT_IN_MSEC)
		{
			DriverStation.reportWarning("=!=!= HangGearInTeleopSequence Timeout ABORT =!=!=", false);
			return false;
		}
		
		_chassis.Drive(DRIVE_BACKWARDS_SPEED, 0);
		_gearHandler.SpinInfeedWheelsVBus(GEAR_OUTFEED_SPEED);
		_gearHandler.MoveTiltAxisVBus(.025);    // needs to be set to position in order to execute with encoder functionality
		
		//TODO: Implement (probably) elapsed time based functionality
		//			set _isStillRunning to false when complete!
		
		return _isStillRunning;
	}

	//============================================================================================
	// Properties follow
	//============================================================================================
}
