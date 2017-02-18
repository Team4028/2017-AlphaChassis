package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.LogData;
import org.usfirst.frc.team4028.robot.Utilities;
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality for the SHOOTER (& Blender) Subsystem
//=====> For Changes see Prat Bruns

//-------------------------------------------------------------
//	Rev		By			D/T				Description
//	0		Patrick		2/16 8:47		Enabling Blender and Feeder Motors
//	1		Patrick		2/18 5:36		Code Review
//-------------------------------------------------------------
public class Shooter 
{
	// =====================================================================
	// 5 DC Motors
	//		1 Talon w/ Encoder, 	PID V Mode		2nd Stage
	//		1 Talon w/ Encoder, 	PID V Mode		1st Stage
	//		1 Talon w/o Encoder,	% VBus Mode		Feed Motor
	//		1 Talon w/ Encoder,		% VBus Mode		Blender
	//		1 Talon w/ Encoder,		PID P Mode		Turret
	//
	// 1 Servo
	// 		I Linear Actuator		PWM				Slider
	// =====================================================================
	
	// define class level variables for Robot objects`
	private CANTalon _firstStgMtr;
	private CANTalon _secondStageMtr;
	private CANTalon _blenderMtr;
	private CANTalon _feederMtr;
	
	private PWM _linearActuator;
	private double _currentSliderPosition;
	
	// define class level working variables
	private int _stg1MtrTargetRPM;
	private int _stg2MtrTargetRPM;
	
	//define class level PID constants
	private static final double FIRST_STAGE_MTG_FF_GAIN = 0.035;
	private static final double FIRST_STAGE_MTG_P_GAIN = 0.075;
	private static final double FIRST_STAGE_MTG_I_GAIN = 0.0;
	private static final double FIRST_STAGE_MTG_D_GAIN = 0.0;
	
	private static final double SECOND_STAGE_MTG_FF_GAIN = 0.029;
	private static final double SECOND_STAGE_MTG_P_GAIN = 0.075;
	private static final double SECOND_STAGE_MTG_I_GAIN = 0.0;
	private static final double SECOND_STAGE_MTG_D_GAIN = 0.0;
	
	//define class level Actuator Constants
	private static final double MAX_THRESHOLD_ACTUATOR = 0.7; 
	private static final double MIN_THRESHOLD_ACTUATOR = 0.4;
	private static final double CHANGE_INTERVAL_ACTUATOR = 0.025;
	private static final double INITIAL_POSITION_ACTUATOR = 0.4;
	
	//============================================================================================
	// CONSTRUCTORS FOLLOW
	//============================================================================================
	public Shooter(int firstStgMtrCanBusAddr, int secondStageMtrCanBusAddr, int blenderMtrCanBusAddr, 
				   int feederMtrCanBusAddr, int sliderPWMPort)
	{
		// First Stage Motor
		_firstStgMtr = new CANTalon(firstStgMtrCanBusAddr);
		_firstStgMtr.changeControlMode(CANTalon.TalonControlMode.Speed);	// open loop throttle
		_firstStgMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_firstStgMtr.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);	// set encoder to be feedback device
    	_firstStgMtr.reverseSensor(true);  							// do not invert encoder feedback
		_firstStgMtr.enableLimitSwitch(false, false);
        // set the peak and nominal outputs, 12V means full 
		_firstStgMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		_firstStgMtr.configPeakOutputVoltage(12.0f, 0.0f);
    	
		// set closed loop gains in slot0 
		_firstStgMtr.setProfile(0);
		_firstStgMtr.setF(FIRST_STAGE_MTG_FF_GAIN); 
		_firstStgMtr.setP(FIRST_STAGE_MTG_P_GAIN); 
		_firstStgMtr.setI(FIRST_STAGE_MTG_I_GAIN); 
		_firstStgMtr.setD(FIRST_STAGE_MTG_D_GAIN);
				
		// Second Stage Motor
		_secondStageMtr = new CANTalon(secondStageMtrCanBusAddr);
		_secondStageMtr.changeControlMode(CANTalon.TalonControlMode.Speed);	// open loop throttle
		_secondStageMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_secondStageMtr.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);	// set encoder to be feedback device
    	_secondStageMtr.reverseSensor(true);  							// do not invert encoder feedback
		_secondStageMtr.enableLimitSwitch(false, false);
    	//_secondStageMtr.reverseOutput(true);
        // set the peak and nominal outputs, 12V means full
		_secondStageMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		_secondStageMtr.configPeakOutputVoltage(12.0f, 0.0f);
		
		// set closed loop gains in slot0
		_secondStageMtr.setProfile(0);
		_secondStageMtr.setF(SECOND_STAGE_MTG_FF_GAIN); 
		_secondStageMtr.setP(SECOND_STAGE_MTG_P_GAIN); 
		_secondStageMtr.setI(SECOND_STAGE_MTG_I_GAIN); 
		_secondStageMtr.setD(SECOND_STAGE_MTG_D_GAIN);
		
		// Blender Motor
		_blenderMtr = new CANTalon(blenderMtrCanBusAddr);
		_blenderMtr.enableBrakeMode(false);
		_blenderMtr.enableLimitSwitch(false, false);
		
		// Feeder Motor
		_feederMtr = new CANTalon(feederMtrCanBusAddr);
		_feederMtr.enableBrakeMode(false);
		_feederMtr.enableLimitSwitch(false, false);
		
		// Slider
		_linearActuator = new PWM(sliderPWMPort);
	}
	
	//============================================================================================
	// METHODS FOLLOW
	//============================================================================================
	
	public void FullStop() 
	{
		SpinStg1Wheel(0);
		SpinStg2Wheel(0);
		SpinBlender(0);
		SpinFeeder(0);
	}
	
	//============================================================================================
	// Shooter Motors
	//============================================================================================

	public void SpinStg1Wheel(int targetRPM)
	{
		_stg1MtrTargetRPM = targetRPM;
		
		_firstStgMtr.set(_stg1MtrTargetRPM);
	}
	
	public void SpinStg2Wheel(int targetRPM)
	{
		_stg2MtrTargetRPM = targetRPM;
		
		_secondStageMtr.set(_stg2MtrTargetRPM);
	}
	
	//============================================================================================
	// Blender/Feeder Motors
	//============================================================================================
	
	public void SpinBlender(double blenderVbusCommand)
	{
		_blenderMtr.set(blenderVbusCommand);
	}
	
	public void SpinFeeder(double feederVbusCommand)
	{
		_feederMtr.set(feederVbusCommand);
	}
		
	//============================================================================================
	// Linear Actuator
	//============================================================================================
	
	public void ActuatorInitialConfig()
	{
		_linearActuator.setPosition(INITIAL_POSITION_ACTUATOR);
		_currentSliderPosition = INITIAL_POSITION_ACTUATOR;
	} 
	
	public void ActuatorUp()
	{
		if (_currentSliderPosition < MAX_THRESHOLD_ACTUATOR)
		{
			_currentSliderPosition += CHANGE_INTERVAL_ACTUATOR;
			_currentSliderPosition = Utilities.RoundDouble(_currentSliderPosition, 3); //rounds to 3 Decimal Places
			_linearActuator.setPosition(_currentSliderPosition);
		}
		else
		{
			DriverStation.reportWarning("Actuator Already at Maximum Position", true);
		}
	}
	
	public void ActuatorDown()
	{
		if (_currentSliderPosition > MIN_THRESHOLD_ACTUATOR)
		{
			_currentSliderPosition -= CHANGE_INTERVAL_ACTUATOR;
			_currentSliderPosition = Utilities.RoundDouble(_currentSliderPosition, 3); //rounds to 3 Decimal Places
			_linearActuator.setPosition(_currentSliderPosition);
		}
		else
		{
			DriverStation.reportWarning("Actuator Already at Minimum Position", true);
		}
	}
	
	//============================================================================================
	// Update Smart Dashboard with Current Values
	//============================================================================================	
	
	public void OutputToSmartDashboard()
	{
		//Display Current Actuator Value
		String outData = "?";
		outData = String.format( "%.3f", _currentSliderPosition); //Outputs "Max" and "Min" at respective values
		if(_currentSliderPosition == MAX_THRESHOLD_ACTUATOR)
		{
			outData = outData + " (MAX)";
		}
		else if(_currentSliderPosition == MIN_THRESHOLD_ACTUATOR)
		{
			outData = outData + " (MIN)";
		}
		SmartDashboard.putString("Actuator Current Value", outData);
	}
	
	//============================================================================================
	// Update Logging File
	//============================================================================================	
	
	public void UpdateLogData(LogData logData)
	{
		logData.AddData("Stg1Mtr:Cmd_Rpm", String.format("%d", _stg1MtrTargetRPM));
		logData.AddData("Stg1Mtr:Act_Rpm", String.format("%d", getStg1ActualRPM()));
		logData.AddData("Stg1Mtr:Err_%", String.format("%.2f%%", getStg1RPMErrorPercent()));
		
		logData.AddData("Stg2Mtr:Cmd_Rpm", String.format("%d", _stg2MtrTargetRPM));
		logData.AddData("Stg2Mtr:Act_Rpm", String.format("%d", getStg2ActualRPM()));	
		logData.AddData("Stg2Mtr:Err_%", String.format("%.2f%%", getStg2RPMErrorPercent()));
		
		logData.AddData("Actuator Position", String.format("%.3f", _currentSliderPosition));
	}
	
	//============================================================================================
	// PROPERTY ACCESSORS FOLLOW
	//============================================================================================
	private double getStg1ActualRPM()
	{
		return _firstStgMtr.getSpeed();
	}
	
	private double getStg1RPMErrorPercent()
	{
		if(Math.abs(_stg1MtrTargetRPM) > 0 )
		{		
			return ((_stg1MtrTargetRPM - getStg1ActualRPM()) / _stg1MtrTargetRPM) * 100.0 * -1.0;
		}
		else
		{
			return 0.0;
		}
	}
	
	private double getStg2ActualRPM()
	{
		return _secondStageMtr.getSpeed();
	}
	
	private double getStg2RPMErrorPercent()
	{
		if(Math.abs(_stg2MtrTargetRPM) > 0 )
		{		
			return ((_stg2MtrTargetRPM - getStg2ActualRPM()) / _stg2MtrTargetRPM) * 100.0 * -1.0;
		}
		else
		{
			return 0.0;
		}
	}
}
