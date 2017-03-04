package org.usfirst.frc.team4028.robot.autonRoutines;

import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.sequences.HangGearInTeleopSequence;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.util.GeneratedTrajectory;

import edu.wpi.first.wpilibj.DriverStation;

//this class implements the logic for the simple "Hang the Gear on the Boiler Side Peg" auton
//------------------------------------------------------
//Rev		By		 	D/T			Desc
//===		========	===========	=================================
//0			Sebas	 	25.Feb.2017	Initial Version
//1.0 		Sebas 		4.Mar.2017	Added Motion Profile + Hang Gear
//------------------------------------------------------
//=====> For Changes see Sebas
public class HangBoilerSideGear {
	// define class level variables for Robot subsystems
	private GearHandler _gearHandler;
	private Chassis _chassis;
	private NavXGyro _navX;
	private TrajectoryDriveController _trajController;
	private HangGearInTeleopSequence _hangGearInTeleopSeq;
	
	private enum AUTON_STATE {
		UNDEFINED,
		MOVE_TO_TARGET,
		INIT_GEAR_SEQUENCE,
		RUN_GEAR_SEQUENCE
	}
	
	// define class level working variables
	private long _autonStartedTimeStamp;
	private boolean _isStillRunning;
	
	private AUTON_STATE _autonState;
	
	// define class level constants
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public HangBoilerSideGear(GearHandler gearHandler, Chassis chassis, NavXGyro navX, HangGearInTeleopSequence hangGear) {
		// these are the subsystems that this auton routine needs to control
		_gearHandler = gearHandler;
		_chassis = chassis;
		_navX = navX;
		_hangGearInTeleopSeq = hangGear;
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
		_autonState = AUTON_STATE.MOVE_TO_TARGET;
		
		_trajController.loadProfile(GeneratedTrajectory.LeftPoints, GeneratedTrajectory.RightPoints, 1.0, -1.0);
		_trajController.enable();
		DriverStation.reportError(Double.toString(_trajController.getCurrentHeading()), false);
		DriverStation.reportWarning("===== Entering HangBoilerSideGear Auton =====", false);
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
    	} else {
    		DriverStation.reportError("Zeroed", false);
    		_gearHandler.MoveGearToScorePosition();
    	}
      	
      	switch (_autonState) {
      		case MOVE_TO_TARGET:
      			if (_trajController.onTarget()) {
      				_trajController.disable();
      				DriverStation.reportError(Double.toString(_trajController.getCurrentHeading()), false);
      				_autonState = AUTON_STATE.INIT_GEAR_SEQUENCE;
      			}
      			break;
      			
      		case INIT_GEAR_SEQUENCE:
      			_hangGearInTeleopSeq.Initialize();
      			_autonState = AUTON_STATE.RUN_GEAR_SEQUENCE;
      			break;
      			
      		case RUN_GEAR_SEQUENCE:
      			boolean isStillRunning = _hangGearInTeleopSeq.ExecuteSequenceRentrant();
      			if (!isStillRunning) {
      				DriverStation.reportError("Done", false);
      			}
      			break;
      			
      		case UNDEFINED:
      			break;
      	}
		// cleanup
		if(!_isStillRunning) {
			DriverStation.reportWarning("===== Completed HangBoilerSideGear Auton =====", false);
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