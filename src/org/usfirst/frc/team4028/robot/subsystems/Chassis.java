package org.usfirst.frc.team4028.robot.subsystems;

import java.util.TimerTask;

import org.usfirst.frc.team4028.robot.util.GeneratedTrajectory;
import org.usfirst.frc.team4028.robot.LogData;
import org.usfirst.frc.team4028.robot.constants.RobotMap;
import org.usfirst.frc.team4028.robot.controllers.ChassisAutoAimController;
import org.usfirst.frc.team4028.robot.controllers.TrajectoryDriveController;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

//This class implements all functionality for the GEAR Subsystem
//
//------------------------------------------------------
//	Rev		By		 	D/T			Desc
//	===		========	===========	=================================
//  1.0		Seabass		2/25/17		Initial 
//------------------------------------------------------
//
//=====> For Changes see Sebastian Rodriguez
public class Chassis 
{
	// =====================================================================
	// 4 DC Motors
	//		1 Talon w/ Encoder		Left Master
	//		1 Talon w/o Encoder		Left Slave
	//		1 Talon w/ Encoder		Right Master
	//		1 Talon w/o Encoder		Right Slave
	//
	// 1 Solenoid
	// 		1 Dual Action 			Shifter
	// =====================================================================
	
	// define class level variables for Robot objects
	private CANTalon _leftDriveMaster, _leftDriveSlave, _rightDriveMaster, _rightDriveSlave;
	private RobotDrive _robotDrive;				// this supports arcade/tank style drive controls
	private DoubleSolenoid _shifterSolenoid;
	
	// define class level variables to hold state
	private Value _shifterSolenoidPosition;
	private long _lastCmdChgTimeStamp;
	private double _driveSpeedScalingFactorClamped;
	private boolean _isBrakeMode = false;
	
	//accel decel variables
	private boolean _isAccelDecelEnabled;
	private double _currentThrottleCmdScaled;
	private double _currentThrottleCmdAccDec;
	private double _previousThrottleCmdScaled;
	private double _previousThrottleCmdAccDec;
	
	private double _arcadeDriveThrottleCmdAdj;
	private double _arcadeDriveTurnCmdAdj;
	
	private static final double ACC_DEC_RATE_FACTOR = 5.0;
	private static final double ACC_DEC_TOTAL_TIME_SECS = 0.8;
	
	private static final double _turnSpeedScalingFactor = 0.7;
	
	// Gearbox Ratios: 1:3 encoder shaft, 34:50 output shaft
	
	// define public enums exposed by this class
	public enum GearShiftPosition {
		UNKNOWN,
		HIGH_GEAR,
		LOW_GEAR
	}	
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public Chassis(int talonLeftMasterCanBusAddr, int talonLeftSlave1CanBusAddr,
					int talonRightMasterCanBusAddr, int talonRightSlave1CanBusAddr,
					int pcmCanBusAddress, 
					int shifterSolenoidHighGearPCMPort, int shifterSolenoidLowGearPCMPort)
		
	{
    	// ===================
    	// Left Drive Motors, Tandem Pair, looking out motor shaft: CW = Drive FWD
    	// ===================
    	_leftDriveMaster = new CANTalon(talonLeftMasterCanBusAddr);
    	_leftDriveMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle
    	_leftDriveMaster.enableBrakeMode(_isBrakeMode);							// default to brake mode DISABLED
    	_leftDriveMaster.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
    	_leftDriveMaster.configEncoderCodesPerRev(1850);
    	_leftDriveMaster.reverseSensor(false);  							// do not invert encoder feedback
    	_leftDriveMaster.enableLimitSwitch(false, false);

		_leftDriveSlave = new CANTalon(talonLeftSlave1CanBusAddr);
	   	_leftDriveSlave.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
	   	_leftDriveSlave.set(talonLeftMasterCanBusAddr);
	   	_leftDriveSlave.enableBrakeMode(_isBrakeMode);							// default to brake mode DISABLED
	    _leftDriveSlave.enableLimitSwitch(false, false);

    	// ===================
    	// Right Drive Motors, Tandem Pair, looking out motor shaft: CW = Drive FWD
    	// ===================
		_rightDriveMaster = new CANTalon(talonRightMasterCanBusAddr);
		_rightDriveMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle
		_rightDriveMaster.enableBrakeMode(_isBrakeMode);							// default to brake mode DISABLED
    	_rightDriveMaster.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
    	_rightDriveMaster.configEncoderCodesPerRev(1850);
    	_rightDriveMaster.reverseSensor(true);  							// do not invert encoder feedback
		_rightDriveMaster.enableLimitSwitch(false, false);

		_rightDriveSlave = new CANTalon(talonRightSlave1CanBusAddr);
		_rightDriveSlave.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
		_rightDriveSlave.set(talonRightMasterCanBusAddr);
		_rightDriveSlave.enableBrakeMode(_isBrakeMode);							// default to brake mode DISABLED
		_rightDriveSlave.enableLimitSwitch(false, false);
    	  	
    	//====================
    	// Shifter
    	//====================
    	_shifterSolenoid = new DoubleSolenoid(pcmCanBusAddress, shifterSolenoidHighGearPCMPort, shifterSolenoidLowGearPCMPort);
    	
    	//====================
    	// Arcade Drive
    	//====================
    	// Arcade Drive configured to drive in "2 motor per side setup, 
    	//	other motors follow master as slaves 
    	_robotDrive = new RobotDrive(_leftDriveMaster, _rightDriveMaster);
    
    	//set default scaling factor
    	_driveSpeedScalingFactorClamped = 1.0;
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	
	// This is the (arcade) main drive method
	public void ArcadeDrive(double newThrottleCmdRaw, double newTurnCmdRaw) {
		// ----------------
		// Step 1: make sure we are in %VBus mode (we may have chg'd to PID mode)
		// ----------------
		if(_leftDriveMaster.getControlMode() != CANTalon.TalonControlMode.PercentVbus) {
			_leftDriveMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		}
		
		if(_rightDriveMaster.getControlMode() != CANTalon.TalonControlMode.PercentVbus) {
			_rightDriveMaster.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		}
		
		// calc scaled throttle cmds
		double newThrottleCmdScaled = newThrottleCmdRaw * _driveSpeedScalingFactorClamped;
		double newTurnCmdScaled = newTurnCmdRaw * _turnSpeedScalingFactor;
		
		// if the cmd just chg'd reset 
		if(newThrottleCmdScaled != _previousThrottleCmdScaled) {
			_previousThrottleCmdScaled = _currentThrottleCmdAccDec;
			_currentThrottleCmdScaled = newThrottleCmdScaled;
			
			_lastCmdChgTimeStamp = System.currentTimeMillis();
		}
			
		// if acc/dec mode is enabled
		if(_isAccelDecelEnabled) {
			_previousThrottleCmdAccDec = _currentThrottleCmdAccDec;
			
			//implement speed scaling
			_arcadeDriveThrottleCmdAdj = calcAccelDecelThrottleCmd(_currentThrottleCmdScaled, _previousThrottleCmdScaled, _lastCmdChgTimeStamp);
			
			_currentThrottleCmdAccDec = _arcadeDriveThrottleCmdAdj;
			
			if(Math.abs(_arcadeDriveThrottleCmdAdj - _currentThrottleCmdScaled) < 0.1) {
				_previousThrottleCmdScaled = _currentThrottleCmdScaled;
			}
		}
		else {
			_arcadeDriveThrottleCmdAdj = newThrottleCmdScaled;
		}
		
		_arcadeDriveTurnCmdAdj = newTurnCmdScaled;
		
		// send cmd to mtr controllers
		_robotDrive.arcadeDrive(_arcadeDriveThrottleCmdAdj, _arcadeDriveTurnCmdAdj);		
	}
	
	public void TankDrive(double leftCmd, double rightCmd) {
		_robotDrive.tankDrive(leftCmd, rightCmd);
	}
	
	// stop the motors
	public void FullStop() {
		ArcadeDrive(0.0, 0.0);
	}
	
	// shifts between high & low gear
	public void ShiftGear(GearShiftPosition gear) {
		// send cmd to to solenoids
		switch(gear) {
			case HIGH_GEAR:
				_shifterSolenoid.set(RobotMap.SHIFTER_SOLENOID_HIGH_GEAR_POSITION);
				_shifterSolenoidPosition = RobotMap.SHIFTER_SOLENOID_HIGH_GEAR_POSITION;
				
    			DriverStation.reportError("Shift into HIGH gear", false);
				break;
			
			case LOW_GEAR:
				_shifterSolenoid.set(RobotMap.SHIFTER_SOLENOID_LOW_GEAR_POSITION);
				_shifterSolenoidPosition = RobotMap.SHIFTER_SOLENOID_LOW_GEAR_POSITION;
				
    			DriverStation.reportError("Shift into LOW gear", false);
				break;
		}
	}
	
	public void ZeroDriveEncoders() {
		_leftDriveMaster.setPosition(0);
		_rightDriveMaster.setPosition(0);
	}
	
	// update the Dashboard with any Chassis specific data values
	public void OutputToSmartDashboard() {
		
	}
	
	public void UpdateLogData(LogData logData) {
		logData.AddData("Chassis:LeftDriveMtrSpd", String.format("%.2f", _leftDriveMaster.getSpeed()));
		logData.AddData("Chassis:LeftDriveMtr%VBus", String.format("%.2f", _leftDriveMaster.getOutputVoltage()/_leftDriveMaster.getBusVoltage()));
		logData.AddData("Chassis:LeftDriveMtrPos", String.format("%.0f", _leftDriveMaster.getPosition()));
		
		logData.AddData("Chassis:RightDriveMtrSpd", String.format("%.2f", _rightDriveMaster.getSpeed()));
		logData.AddData("Chassis:RightDriveMtr%VBus", String.format("%.2f", _rightDriveMaster.getOutputVoltage()/_rightDriveMaster.getBusVoltage()));
		logData.AddData("Chassis:RightDriveMtrPos", String.format("%.0f", _rightDriveMaster.getPosition()));
	}
	
	//============================================================================================
	// Property Accessors follow
	//============================================================================================
	
	// Returns the current shifter position (gear)
	public GearShiftPosition getGearShiftPosition() {
		if(_shifterSolenoidPosition == RobotMap.SHIFTER_SOLENOID_HIGH_GEAR_POSITION)
			return GearShiftPosition.HIGH_GEAR;
		else if(_shifterSolenoidPosition == RobotMap.SHIFTER_SOLENOID_LOW_GEAR_POSITION)
			return GearShiftPosition.LOW_GEAR;
		else
			return GearShiftPosition.UNKNOWN;		
	}
	
	public void setDriveSpeedScalingFactor(double speedScalingFactor) {
		// for safety, clamp the scaling factor to max of +1, -1
		if (speedScalingFactor > 1.0) {
			speedScalingFactor = 1.0;
		}
		else if (speedScalingFactor < -1.0){
			speedScalingFactor = -1.0;
		}
		
		_driveSpeedScalingFactorClamped = speedScalingFactor;
	}
	
	public void setIsAccDecModeEnabled(boolean isEnabled) {
		_isAccelDecelEnabled = isEnabled;
		DriverStation.reportWarning("===== Acc/Dec Mode Enabled? " + isEnabled, false);
	}
	
	public boolean getIsAccDecModeEnabled() {
		return _isAccelDecelEnabled;
	}
	
	public double getHeadingInDegrees() {
		return 1.0; // Add heading getter here
	}
	
	public double getLeftEncoderCurrentPosition() {
		return _leftDriveMaster.getPosition();
	}
	
	public double getLeftEncoderCurrentVelocity() {
		return _leftDriveMaster.getEncVelocity();
	}
	
	public double getRightEncoderCurrentPosition() {
		return _rightDriveMaster.getPosition();
	}
	
	public double getRightEncoderCurrentVelocity() {
		return _rightDriveMaster.getEncVelocity();
	}
	
	//============================================================================================
	// Utility Helper Methods
	//============================================================================================
	// implement s-curve accel / decel
	private double calcAccelDecelThrottleCmd(double currentThrottleCmd, double previousThrottleCmd, long lastCmdChgTimeStamp) {
		double accDecMidpointTimeSecs = ACC_DEC_TOTAL_TIME_SECS / 2.0;    // a

        double minusK = -1.0 * ACC_DEC_RATE_FACTOR;
        double elapsedSecsSinceLastChg = (System.currentTimeMillis() - _lastCmdChgTimeStamp) / 1000.0; // x
        double xMinusA = elapsedSecsSinceLastChg - accDecMidpointTimeSecs;

        double scaleFactor = 1.0 / ( 1.0 + Math.exp(minusK * xMinusA) );

        // finally calc the adj cmd
        double accDecCmd = previousThrottleCmd + ((_currentThrottleCmdScaled - previousThrottleCmd) * scaleFactor);
        
        //DriverStation.reportError("accDecCmd = " + Double.toString(accDecCmd), false);
        return accDecCmd;
	}
}
