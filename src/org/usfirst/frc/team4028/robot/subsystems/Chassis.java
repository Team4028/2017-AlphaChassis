package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.LogData;
import org.usfirst.frc.team4028.robot.constants.RobotMap;

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
	private CANTalon _leftDriveMasterMtr;
	private CANTalon _leftDriveSlave1Mtr;
	private CANTalon _rightDriveMasterMtr;
	private CANTalon _rightDriveSlave1Mtr;

	private RobotDrive _robotDrive;				// this supports arcade style drive controls
	
	private DoubleSolenoid _shifterSolenoid;
	
	// define class level variables to hold state
	private Value _shifterSolenoidPosition;
	private long _lastCmdChgTimeStamp;
	private double _driveSpeedScalingFactorClamped;
	
	//accel decel variables
	private boolean _isAccelDecelEnabled;
	private double _currentThrottleCmdScaled;
	private double _currentThrottleCmdAccDec;
	private double _previousThrottleCmdScaled;
	private double _previousThrottleCmdAccDec;
	
	private double _arcadeDriveThrottleCmdAdj;
	private double _arcadeDriveTurnCmdAdj;
	
	
	private static final double ACC_DEC_RATE_FACTOR = 5.0;
	private static final double ACC_DEC_TOTAL_TIME_SECS = 2.0;
	
	private static final double _turnSpeedScalingFactor = 0.6;
	
	// define public enums exposed by this class
	public enum GearShiftPosition
	{
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
    	_leftDriveMasterMtr = new CANTalon(talonLeftMasterCanBusAddr);
    	_leftDriveMasterMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle
    	_leftDriveMasterMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	//_leftDriveMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
    	//_leftDriveMasterMtr.reverseSensor(false);  							// do not invert encoder feedback
    	_leftDriveMasterMtr.enableLimitSwitch(false, false);
    	//_leftDriveMasterMtr.reverseOutput(true);
		    	
		_leftDriveSlave1Mtr = new CANTalon(talonLeftSlave1CanBusAddr);
	   	_leftDriveSlave1Mtr.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
	   	_leftDriveSlave1Mtr.set(talonLeftMasterCanBusAddr);
	   	_leftDriveSlave1Mtr.enableBrakeMode(false);							// default to brake mode DISABLED
	    _leftDriveSlave1Mtr.enableLimitSwitch(false, false);
	    //_leftDriveSlaveMtr.reverseOutput(true);
    	    	   	   	
    	// ===================
    	// Right Drive Motors, Tandem Pair, looking out motor shaft: CW = Drive FWD
    	// ===================
		_rightDriveMasterMtr = new CANTalon(talonRightMasterCanBusAddr);
		_rightDriveMasterMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle
		_rightDriveMasterMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	//_rightDriveMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
    	//_rightDriveMasterMtr.reverseSensor(false);  							// do not invert encoder feedback
		_rightDriveMasterMtr.enableLimitSwitch(false, false);
    	//_rightDriveMasterMtr.reverseOutput(true);
    	   	  	
		_rightDriveSlave1Mtr = new CANTalon(talonRightSlave1CanBusAddr);
		_rightDriveSlave1Mtr.changeControlMode(CANTalon.TalonControlMode.Follower);	// set this mtr ctrlr as a slave
		_rightDriveSlave1Mtr.set(talonRightMasterCanBusAddr);
		_rightDriveSlave1Mtr.enableBrakeMode(false);							// default to brake mode DISABLED
		_rightDriveSlave1Mtr.enableLimitSwitch(false, false);
	   	//_rightDriveSlave1Mtr.reverseOutput(true);
    	  	
    	//====================
    	// Shifter
    	//====================
    	_shifterSolenoid = new DoubleSolenoid(pcmCanBusAddress, shifterSolenoidHighGearPCMPort, shifterSolenoidLowGearPCMPort);
    	
    	//====================
    	// Arcade Drive
    	//====================
    	// Arcade Drive configured to drive in "2 motor per side setup, 
    	//	other motors follow master as slaves 
    	_robotDrive = new RobotDrive(_leftDriveMasterMtr, _rightDriveMasterMtr);
    
    	//set default scaling factor
    	_driveSpeedScalingFactorClamped = 1.0;
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	
	// This is the (arcade) main drive method
	public void Drive(double newThrottleCmdRaw, double newTurnCmdRaw)
	{
		// ----------------
		// Step 1: make sure we are in %VBus mode (we may have chg'd to PID mode)
		// ----------------
		if(_leftDriveMasterMtr.getControlMode() != CANTalon.TalonControlMode.PercentVbus)
		{
			_leftDriveMasterMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		}
		
		if(_rightDriveMasterMtr.getControlMode() != CANTalon.TalonControlMode.PercentVbus)
		{
			_rightDriveMasterMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		}
		
		// calc scaled throttle cmds
		double newThrottleCmdScaled = newThrottleCmdRaw * _driveSpeedScalingFactorClamped;
		double newTurnCmdScaled = newTurnCmdRaw * _turnSpeedScalingFactor;
		
		// if the cmd just chg'd reset 
		if(newThrottleCmdScaled != _previousThrottleCmdScaled)
		{
			_previousThrottleCmdScaled = _currentThrottleCmdAccDec;
			_currentThrottleCmdScaled = newThrottleCmdScaled;
			
			_lastCmdChgTimeStamp = System.currentTimeMillis();
		}
			
		// if acc/dec mode is enabled
		if(_isAccelDecelEnabled)
		{
			_previousThrottleCmdAccDec = _currentThrottleCmdAccDec;
			
			//implement speed scaling
			_arcadeDriveThrottleCmdAdj = CalcAccelDecelThrottleCmd(_currentThrottleCmdScaled, _previousThrottleCmdScaled, _lastCmdChgTimeStamp);
			
			_currentThrottleCmdAccDec = _arcadeDriveThrottleCmdAdj;
			
			if(Math.abs(_arcadeDriveThrottleCmdAdj - _currentThrottleCmdScaled) < 0.1)
			{
				_previousThrottleCmdScaled = _currentThrottleCmdScaled;
			}
		}
		else
		{
			_arcadeDriveThrottleCmdAdj = newThrottleCmdScaled;
		}
		
		_arcadeDriveTurnCmdAdj = newTurnCmdScaled;
		
		// send cmd to mtr controllers
		_robotDrive.arcadeDrive(_arcadeDriveThrottleCmdAdj, _arcadeDriveTurnCmdAdj);		
	}
	
	// stop the motors
	public void FullStop()
	{
		Drive(0.0, 0.0);
	}
	
	// shifts between high & low gear
	public void ShiftGear(GearShiftPosition gear)
	{
		// send cmd to to solenoids
		switch(gear)
		{
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
	
	public void ZeroDriveEncoders()
	{
		_leftDriveMasterMtr.setPosition(0);
		_rightDriveMasterMtr.setPosition(0);
	}
	
	// update the Dashboard with any Chassis specific data values
	public void OutputToSmartDashboard()
	{
		
	}
	
	public void UpdateLogData(LogData logData)
	{
		logData.AddData("Chassis:LeftDriveMtrSpd", String.format("%.2f", _leftDriveMasterMtr.getSpeed()));
		logData.AddData("Chassis:LeftDriveMtr%VBus", String.format("%.2f", _leftDriveMasterMtr.getOutputVoltage()/_leftDriveMasterMtr.getBusVoltage()));
		logData.AddData("Chassis:LeftDriveMtrPos", String.format("%.0f", _leftDriveMasterMtr.getPosition()));
		
		logData.AddData("Chassis:RightDriveMtrSpd", String.format("%.2f", _rightDriveMasterMtr.getSpeed()));
		logData.AddData("Chassis:RightDriveMtr%VBus", String.format("%.2f", _rightDriveMasterMtr.getOutputVoltage()/_rightDriveMasterMtr.getBusVoltage()));
		logData.AddData("Chassis:RightDriveMtrPos", String.format("%.0f", _rightDriveMasterMtr.getPosition()));
	}
	
	//============================================================================================
	// Property Accessors follow
	//============================================================================================
	
	// Returns the current shifter position (gear)
	public GearShiftPosition getGearShiftPosition()
	{
		if(_shifterSolenoidPosition == RobotMap.SHIFTER_SOLENOID_HIGH_GEAR_POSITION)
			return GearShiftPosition.HIGH_GEAR;
		else if(_shifterSolenoidPosition == RobotMap.SHIFTER_SOLENOID_LOW_GEAR_POSITION)
			return GearShiftPosition.LOW_GEAR;
		else
			return GearShiftPosition.UNKNOWN;		
	}
	
	public void setDriveSpeedScalingFactor(double speedScalingFactor)
	{
		// for safety, clamp the scaling factor to max of +1, -1
		if (speedScalingFactor > 1.0){
			speedScalingFactor = 1.0;
		}
		else if (speedScalingFactor < -1.0){
			speedScalingFactor = -1.0;
		}
		
		_driveSpeedScalingFactorClamped = speedScalingFactor;
	}
	
	public void setIsAccDecModeEnabled(boolean isEnabled)
	{
		_isAccelDecelEnabled = isEnabled;
		
		DriverStation.reportWarning("===== Acc/Dec Mode Enabled? " + isEnabled, false);
	}
	
	public boolean getIsAccDecModeEnabled()
	{
		return _isAccelDecelEnabled;
	}
	
	//============================================================================================
	// Utility Helper Methods
	//============================================================================================
	// implement s-curve accel / decel
	private double CalcAccelDecelThrottleCmd(double currentThrottleCmd, double previousThrottleCmd, long lastCmdChgTimeStamp)
	{
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
