package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.utilities.LogData;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// This class implements all functionality for the GEAR Subsystem
//
//------------------------------------------------------
//	Rev		By		 	D/T				Desc
//	===		========	===========		=================================
//	1		Nick		17.Feb.2017		Initial Version
//------------------------------------------------------
//
// =====> For Changes see Nick Donahue (javadotmakeitwork)
public class GearHandler 
{
	// =====================================================================
	// 2 DC Motors
	//		1 Talon w/ Encoder	w/ Rev Limit Switch		Tilt
	//		1 Talon w/o Encoder							Infeed / Outfeed
	// =====================================================================
	
	// define class level public constants
	public static final double INFEED_TARGET_CMD = -0.5;
	public static final double OUTFEED_TARGET_CMD = 0.5;
	
	// define class level variables for Robot objects
	private CANTalon _gearTiltMotor;
	private CANTalon _gearInfeedMotor;
	
	// define class level private working variables
	private double _targetPositionRotations;
	
	// --------------------------------------------------------
	// define Tilt Motor PID constants
	private static final int 	TILT_PID_P_PROFILE = 0;
	private static final double TILT_PID_P_CONSTANT = 1.6;
	private static final double TILT_PID_I_CONSTANT = 0.0;
	private static final double TILT_PID_D_CONSTANT = 50.0;
	private static final double TILT_PID_RAMP_RATE = 0.1;
		
	private static final double TILT_MAX_V_DOWN_TILT = +2.0; // Down is positive (RIP MAXIMUM...)
	private static final double TILT_MAX_V_UP_TILT = -6.0;
	// --------------------------------------------------------
	
	// --------------------------------------------------------
	// define Working variables and constants for homing the tilt axix
	private enum GEAR_TILT_HOMING_STATE
	{
		UNDEFINED,
		MOVING_TO_HOME,
		AT_HOME,
		TIMEOUT,
		ZEROED
	}
	
	private enum GEAR_TILT_MOVE_LAST_TARGET_POSITION
	{
		UNDEFINED,
		MOVING_TO_SCORING_POSITION,
		MOVING_TO_HOME,
		MOVING_TO_FLOOR
	}
	
	private static final double GEAR_TILT_AXIS_HOME_POSITION_IN_ROTATIONS = 00.00;
	private static final double GEAR_TILT_SCORING_POSITION_IN_ROTATIONS = 000.112;
	private static final double GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_ROTATIONS = 000.373;
	private static final double TARGET_DEADBAND = 00.03;
	
	private static final double GEAR_MOVE_TO_HOME_VELOCITY_CMD = -0.25;   //set
	private static final long GEAR_MAXIMUM_MOVE_TO_HOME_TIME_IN_MSEC = 5000;
	private String _gearTiltState;

	private long _gearTiltAxisStateStartTime;
	private GEAR_TILT_HOMING_STATE _gearTiltAxisZeroCurrentState;
	private GEAR_TILT_MOVE_LAST_TARGET_POSITION _gearTiltMoveLastTargetPosition;
	private boolean _isLastTiltMoveToFloorCallComplete;

	// --------------------------------------------------------
		
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public GearHandler(int talonTiltCanBusAddr, int talonInfeedCanBusAddr)
	{
		// Tilt Motor
		_gearTiltMotor = new CANTalon(talonTiltCanBusAddr);
		_gearTiltMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle
		_gearTiltMotor.enableBrakeMode(true);							// default to brake mode DISABLED
		_gearTiltMotor.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);	// set encoder to be feedback device
		_gearTiltMotor.reverseSensor(false);  							// do not invert encoder feedback
		_gearTiltMotor.enableLimitSwitch(false, true);
		_gearTiltMotor.ConfigRevLimitSwitchNormallyOpen(false);
		
		_gearTiltMotor.setProfile(TILT_PID_P_PROFILE);
		_gearTiltMotor.setP(TILT_PID_P_CONSTANT);
		_gearTiltMotor.setI(TILT_PID_I_CONSTANT);
		_gearTiltMotor.setD(TILT_PID_D_CONSTANT);
		_gearTiltMotor.configNominalOutputVoltage(0.0f, -0.0f);
		_gearTiltMotor.configPeakOutputVoltage(TILT_MAX_V_DOWN_TILT, TILT_MAX_V_UP_TILT);
    	//_gearTiltMotor.reverseOutput(true);
		
		// Infeed Motor
		_gearInfeedMotor = new CANTalon(talonInfeedCanBusAddr);
		_gearInfeedMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle
		_gearInfeedMotor.enableBrakeMode(false);							// default to brake mode DISABLED
    	//_gearInfeedMotor.setFeedbackDevice(FeedbackDevice.QuadEncoder);	// set encoder to be feedback device
    	//_gearInfeedMotor.reverseSensor(false);  							// do not invert encoder feedback
		_gearInfeedMotor.enableLimitSwitch(false, false);
    	//_gearInfeedMotor.reverseOutput(true);
		
		ZeroGearTiltAxisInit();
	}

	//============================================================================================
	// Methods follow
	//============================================================================================	
	
    public void ZeroGearTiltAxisInit()
    {
		_gearTiltMoveLastTargetPosition = GEAR_TILT_MOVE_LAST_TARGET_POSITION.UNDEFINED;   	
		
		// snapshot the current time so we can enforce the timeout
		_gearTiltAxisStateStartTime = System.currentTimeMillis();
    	
    	// did we start on the limit switch? (remember switch is normally closed!)
		if(getIsOnTiltHomeLimtSwitch())
		{
			_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.AT_HOME;
			DriverStation.reportWarning("TiltAxis (Zero State) [INITIAL] ==> [AT_HOME]", false);
		}
		else
		{
			_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.MOVING_TO_HOME;
			DriverStation.reportWarning("TiltAxis (Zero State) [INITIAL] ==> [MOVING_TO_HOME]", false);
		}
    }
	
	// Re-entrant method that will zero the Tilt Axis
    public void ZeroGearTiltAxisReentrant()
    {
    	switch(_gearTiltAxisZeroCurrentState)
    	{    					
    		case MOVING_TO_HOME:
    			// are we on the limit switch? (remember switch is normally closed!
    			if(getIsOnTiltHomeLimtSwitch())
    			{
    				_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.AT_HOME;
    				DriverStation.reportWarning("TiltAxis (Zero State) [MOVING_TO_HOME] ==> [AT_HOME]", false);
    			}
    			else
    			{
    				// check for timeout
    				long elapsedTime = System.currentTimeMillis() - _gearTiltAxisStateStartTime;
    				if (elapsedTime < GEAR_MAXIMUM_MOVE_TO_HOME_TIME_IN_MSEC)
    				{
    					if(_gearTiltMotor.getControlMode() != CANTalon.TalonControlMode.PercentVbus)
    					{
    						_gearTiltMotor.changeControlMode(TalonControlMode.PercentVbus);
    					}
    					_gearTiltMotor.set(GEAR_MOVE_TO_HOME_VELOCITY_CMD);
    				}
    				else
    				{
    					_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.TIMEOUT;
    					DriverStation.reportWarning("TiltAxis (Zero State) [MOVING_TO_HOME] ==> [TIMEOUT]", false);
    				}		
    			}
    			
    			break;
    			
    		case AT_HOME:
    			// chg to PID-Position mode
    			if(_gearTiltMotor.getControlMode() == CANTalon.TalonControlMode.PercentVbus)
    			{
    				_gearTiltMotor.set(0);
    				_gearTiltMotor.changeControlMode(CANTalon.TalonControlMode.Position);
    			}
    			
    			// reset encoder position
    			_gearTiltMotor.setPosition(GEAR_TILT_AXIS_HOME_POSITION_IN_ROTATIONS);
    			
    			// set current target position to be the home position
    			_gearTiltMotor.set(GEAR_TILT_AXIS_HOME_POSITION_IN_ROTATIONS);
    			
    			_gearTiltAxisZeroCurrentState = GEAR_TILT_HOMING_STATE.ZEROED;
    			_gearTiltMoveLastTargetPosition = GEAR_TILT_MOVE_LAST_TARGET_POSITION.MOVING_TO_HOME;
    			DriverStation.reportWarning("TiltAxis (Zero State) [AT_HOME] ==> [ZEROED]", false);
    			break;
    			
    		case TIMEOUT:
    			DriverStation.reportError("Gear Tilt Zero Timed Out", false);
    			break;
    			
    		case UNDEFINED:
    			DriverStation.reportError("Gear Tilt Zero State Undefined", false);
    			break;
    	}
    }
    
    public void MoveGearToHomePosition()
    {
    	MoveTiltAxisPIDP(GEAR_TILT_AXIS_HOME_POSITION_IN_ROTATIONS);
    	_isLastTiltMoveToFloorCallComplete = true;
    }
    
    public void MoveGearToScorePosition()
    {
    	MoveTiltAxisPIDP(GEAR_TILT_SCORING_POSITION_IN_ROTATIONS);
    	_isLastTiltMoveToFloorCallComplete = true;
    }
    
    public void MoveGearToFloorPositionReentrant()
    {
		if(_gearTiltMotor.getPosition() >= (GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_ROTATIONS - TARGET_DEADBAND))
		{
			// gravity fall to floor
			MoveTiltAxisVBus(0.0);
			_isLastTiltMoveToFloorCallComplete = true;
		}
		else
		{
			MoveTiltAxisPIDP(GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_ROTATIONS);
			_isLastTiltMoveToFloorCallComplete = false;
		}
    }
    
    public void MoveTiltAxisPIDP (double positionCmd)
    {
    	if(_gearTiltMotor.getControlMode() != CANTalon.TalonControlMode.Position)
		{
			_gearTiltMotor.changeControlMode(TalonControlMode.Position);
		}
    	_gearTiltMotor.set(positionCmd);
    }

	public void MoveTiltAxisVBus(double percentVBusCmd)
	{
		MoveTiltAxisVBus(percentVBusCmd, false);
	}
    
	public void MoveTiltAxisVBus(double percentVBusCmd, boolean isUseRawCmd)
	{
		if(_gearTiltMotor.getControlMode() != CANTalon.TalonControlMode.PercentVbus)
		{
			_gearTiltMotor.changeControlMode(TalonControlMode.PercentVbus);
		}
		
		if(isUseRawCmd)
		{
			_gearTiltMotor.set(percentVBusCmd);			
		}
		else
		{
			// limit max speed to 25%	=>  * 0.25
			_gearTiltMotor.set(percentVBusCmd * 0.25);	
		}
		
		// cancel any button move that was in process
		_isLastTiltMoveToFloorCallComplete = true;
	}
	
	public void SpinInfeedWheelsVBus(double percentVBusCmd)
	{
		// invert motor command		=>	* -1.0
		// limit max speed to 50%	=>  * 0.50
		_gearInfeedMotor.set(percentVBusCmd * -1.0 * 0.50);
	}
	
	private  String getTiltPosition()
	{
		if(Math.abs(_gearTiltMotor.getPosition()) <= TARGET_DEADBAND)
		{
			_gearTiltState = "Zeroed";
		}
		else if(Math.abs(_gearTiltMotor.getPosition() - GEAR_TILT_SCORING_POSITION_IN_ROTATIONS) <= TARGET_DEADBAND)
		{
			_gearTiltState = "Scoring Position";
		}
		else if(Math.abs(_gearTiltMotor.getPosition() - GEAR_TILT_CHANGE_TO_V_BUS_POSITION_IN_ROTATIONS) <= TARGET_DEADBAND)
		{
			_gearTiltState = "On Floor";
		}
		else
		{
			_gearTiltState = "Moving Between States";
		}
		return _gearTiltState;
	}
		
	public void FullStop()
	{
		MoveTiltAxisVBus(0.0);
		_gearInfeedMotor.set(0.0);
	}
	
	// update the Dashboard with any Climber specific data values
	public void OutputToSmartDashboard()
	{
		SmartDashboard.putString("Gear Tilt Position", String.format("%.3f", _gearTiltMotor.getPosition()));
		SmartDashboard.putString("Gear Tilt State", getTiltPosition());
		SmartDashboard.putString("Gear In/OutFeed Cmd", String.format("%.3f", _gearInfeedMotor.getOutputVoltage()/_gearInfeedMotor.getBusVoltage()));		
	}
	
	public void UpdateLogData(LogData logData)
	{
		logData.AddData("Gear:TiltPos", String.format("%.2f", _gearTiltMotor.getPosition()));
		logData.AddData("Gear:Tilt%VBus", String.format("%.4f", (_gearTiltMotor.getOutputVoltage()) / _gearTiltMotor.getBusVoltage()));
		logData.AddData("Gear:Outfeed%Vbus", String.format("%.2f", _gearTiltMotor.get()));
	}
	
	//============================================================================================
	// Property Accessors follow
	//============================================================================================
	private boolean getIsOnTiltHomeLimtSwitch()
	{
		// remember switch is normally closed!
		return !_gearTiltMotor.isRevLimitSwitchClosed();
	}
	
	public boolean hasTiltAxisBeenZeroed()
	{
		if (_gearTiltAxisZeroCurrentState == GEAR_TILT_HOMING_STATE.ZEROED)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	//allows robot class to check if we are in moving to floor mode so it can keep calling
	public GEAR_TILT_MOVE_LAST_TARGET_POSITION get_gearTiltMoveToPosition()
	{
		return _gearTiltMoveLastTargetPosition;
	}
	
	public boolean IsGearInScoringPosition()
	{
		if(Math.abs(_gearTiltMotor.getPosition() - GEAR_TILT_SCORING_POSITION_IN_ROTATIONS) <= TARGET_DEADBAND)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	
	public boolean getIsLastTiltMoveToFloorCallComplete()
	{
		return _isLastTiltMoveToFloorCallComplete;
	}
}
