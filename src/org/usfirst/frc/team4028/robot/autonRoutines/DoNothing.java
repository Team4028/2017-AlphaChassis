package org.usfirst.frc.team4028.robot.autonRoutines;

import edu.wpi.first.wpilibj.DriverStation;

//this class implements the logic for the simple "Do Nothing" auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	25.Feb.2017	Initial Version
//------------------------------------------------------
//
//=====> For Changes see Sebas
public class DoNothing {
	// define class level variables for Robot subsystems
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private boolean _isStillRunning;
	
	// define class level constants
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public DoNothing() {
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	// execute any logic to initialize this object before ExecuteRentrant is called
	public void Initialize() {
		_autonStartedTimeStamp = System.currentTimeMillis();
		_isStillRunning = false;
		
		DriverStation.reportError("===== Entering DoNothing Auton =====", false);
	}
	
	// execute the auton routine, return = true indicates auton is still running
	// This is a LONG RUNNING method (it spans multiple scan cycles)
	// It is the resonsibility of the caller to repeatable call it until it completes
	public boolean ExecuteRentrant() {
		// cleanup
		if(!_isStillRunning) {
			DriverStation.reportError("===== Completed DoNothing Auton =====", false);
		}
		
		return _isStillRunning; 
	}
	
	//============================================================================================
	// Properties follow
	//============================================================================================
	public boolean getIsStillRunning() {
		return _isStillRunning;
	}
}
