package org.usfirst.frc.team4028.robot.autonRoutines;

import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DriverStation;

//this class implements the logic for the simple "Turn and shoot into the Boiler" auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	25.Feb.2071	Initial Version
//------------------------------------------------------
//
//=====> For Changes see Sebas
public class TurnAndShoot 
{
	// define class level variables for Robot subsystems
	private GearHandler _gearHandler;
	private Shooter _shooter;
	private Chassis _chassis;
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private boolean _isStillRunning;
	
	// define class level constants
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public TurnAndShoot(GearHandler gearHandler, Chassis chassis, Shooter shooter)
	{
		// these are the subsystems that this auton routine needs to control
		_gearHandler = gearHandler;
		_shooter = shooter;
		_chassis = chassis;
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	// execute any logic to initialize this object before ExecuteRentrant is called
	public void Initialize()
	{
		_autonStartedTimeStamp = System.currentTimeMillis();
		_isStillRunning = true;
		
		DriverStation.reportWarning("===== Entering TurnAndShoot Auton =====", false);
	}
	
	// execute the auton routine, return = true indicates auton is still running
	// This is a LONG RUNNING method (it spans multiple scan cycles)
	// It is the resonsibility of the caller to repeatable call it until it completes
	public boolean ExecuteRentrant() 
	{
		// =======================================
		// if not complete, this must run concurrently with all auton routines
		// =======================================
      	if(!_gearHandler.hasTiltAxisBeenZeroed())
    	{
      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
      		//			we must treat it as a Reentrant function
      		//			and automatically recall it until complete
    		_gearHandler.ZeroGearTiltAxisReentrant();
    	}
      	
      	
		// TODO: put auton code here
		
		
		// cleanup
		if(!_isStillRunning)
		{
			DriverStation.reportWarning("===== Completed TurnAndShoot Auton =====", false);
		}
		
		return _isStillRunning; 
	}
	
	//============================================================================================
	// Properties follow
	//============================================================================================
	public boolean getIsStillRunning() 
	{
		return _isStillRunning;
	}
}
