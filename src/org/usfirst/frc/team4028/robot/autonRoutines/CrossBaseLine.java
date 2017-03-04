package org.usfirst.frc.team4028.robot.autonRoutines;

import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.util.GeneratedTrajectory;

import edu.wpi.first.wpilibj.DriverStation;

// this class implements the logic for the simple "Cross the Baseline" auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	25.Feb.2071	Initial Version
//------------------------------------------------------
//
//=====> For Changes see Sebas
public class CrossBaseLine 
{
	// define class level variables for Robot subsystems
	private GearHandler _gearHandler;
	private Chassis _chassis;
	private NavXGyro _navX;
	private TrajectoryDriveController _trajController;
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private boolean _isStillRunning;
	
	// define class level constants
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public CrossBaseLine(GearHandler gearHandler, Chassis chassis, NavXGyro navX) {
		// these are the subsystems that this auton routine needs to control
		_gearHandler = gearHandler;
		_chassis = chassis;
		_navX = navX;
		_trajController = new TrajectoryDriveController(_chassis, _navX);
		_trajController.startTrajectoryController();
		DriverStation.reportError("Auton Initialized", false);
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	// execute any logic to initialize this object before ExecuteRentrant is called
	public void Initialize() {
		_autonStartedTimeStamp = System.currentTimeMillis();
		_isStillRunning = true;
		
		_trajController.loadProfile(GeneratedTrajectory.LeftPoints, GeneratedTrajectory.RightPoints, 1.0, 0);
		_trajController.enable();
		
		DriverStation.reportWarning("===== Entering CrossBaseLine Auton =====", false);
	}
	
	// execute the auton routine, return = true indicates auton is still running
	// This is a LONG RUNNING method (it spans multiple scan cycles)
	// It is the resonsibility of the caller to repeatable call it until it completes
	public boolean ExecuteRentrant() {
		// =======================================
		// if not complete, this must run concurrently with all auton routines
		// =======================================
      	if(!_gearHandler.hasTiltAxisBeenZeroed()) {
      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
      		//			we must treat it as a Reentrant function
      		//			and automatically recall it until complete
    		_gearHandler.ZeroGearTiltAxisReentrant();
      	}   
      	else {
      		_gearHandler.MoveGearToScorePosition();
      	}
      	
		if (_trajController.onTarget()) {
			_trajController.disable();
		}
		
		// cleanup
		if(!_isStillRunning) {
			DriverStation.reportWarning("===== Completed CrossBaseLine Auton =====", false);
		}
		
		return _isStillRunning; 
	}
	
	public void Disabled() {
		_trajController.disable();
	}
	
	//============================================================================================
	// Properties follow
	//============================================================================================
	public boolean getIsStillRunning() {
		return _isStillRunning;
	}
}
