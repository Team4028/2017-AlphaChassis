package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.LogData;
import org.usfirst.frc.team4028.robot.Utilities;
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality for the SHOOTER (& Blender) Subsystem
//=====> For Changes see Prat Bruns

//-------------------------------------------------------------
//	Rev		By			D/T				Description
//	0		Patrick		2/16 8:47		Enabling Blender and Feeder Motors
//	1		Patrick		2/18 5:36		Code Review
//	2		Patrick		2/20 10:02		Code Review on Shooter Testing
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
	private CANTalon _secondStgMtr;
	private CANTalon _blenderMtr;
	private CANTalon _feederMtr;
	
	private Servo _linearActuator;
	private double _currentSliderPosition;
	
	// define class level working variables
	private double _stg1MtrTargetRPM;
	private double _stg2MtrTargetRPM;
	
	//define class level PID constants
	private static final double FIRST_STAGE_MTG_FF_GAIN = 0.036;
	private static final double FIRST_STAGE_MTG_P_GAIN = 0.075;
	private static final double FIRST_STAGE_MTG_I_GAIN = 0.0;
	private static final double FIRST_STAGE_MTG_D_GAIN = 0.0;
	
	private static final double SECOND_STAGE_MTG_FF_GAIN = 0.03125;
	private static final double SECOND_STAGE_MTG_P_GAIN = 0.033;
	private static final double SECOND_STAGE_MTG_I_GAIN = 0.0;
	private static final double SECOND_STAGE_MTG_D_GAIN = 0.0;
	
	//define class level Actuator Constants
	private static final double MAX_THRESHOLD_ACTUATOR = 0.7; 
	private static final double MIN_THRESHOLD_ACTUATOR = 0.4;
	private static final double CHANGE_INTERVAL_ACTUATOR = 0.025;
	private static final double INITIAL_POSITION_ACTUATOR = 0.55;
	
	//define class level Shooter Motor Constants
	private static final double MAX_SHOOTER_RPM = -4400;
	private static final double MIN_SHOOTER_RPM = -3000;
	private static final double FEEDER_PERCENTVBUS_COMMAND = -0.7; //This Motor Needs to Run in Reverse
	private static final double BLENDER_PERCENTVBUS_COMMAND = 0.5;

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
		_firstStgMtr.configPeakOutputVoltage(0.0f, -12.0f);
    	
		// set closed loop gains in slot0 
		_firstStgMtr.setProfile(0);
		_firstStgMtr.setF(FIRST_STAGE_MTG_FF_GAIN); 
		_firstStgMtr.setP(FIRST_STAGE_MTG_P_GAIN); 
		_firstStgMtr.setI(FIRST_STAGE_MTG_I_GAIN); 
		_firstStgMtr.setD(FIRST_STAGE_MTG_D_GAIN);
				
		// Second Stage Motor
		_secondStgMtr = new CANTalon(secondStageMtrCanBusAddr);
		_secondStgMtr.changeControlMode(CANTalon.TalonControlMode.Speed);	// open loop throttle
		_secondStgMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_secondStgMtr.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);	// set encoder to be feedback device
    	_secondStgMtr.reverseSensor(true);  							// do not invert encoder feedback
		_secondStgMtr.enableLimitSwitch(false, false);
    	//_secondStageMtr.reverseOutput(true);
        // set the peak and nominal outputs, 12V means full
		_secondStgMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		_secondStgMtr.configPeakOutputVoltage(0.0f, -12.0f);
		
		// set closed loop gains in slot0
		_secondStgMtr.setProfile(0);
		_secondStgMtr.setF(SECOND_STAGE_MTG_FF_GAIN); 
		_secondStgMtr.setP(SECOND_STAGE_MTG_P_GAIN); 
		_secondStgMtr.setI(SECOND_STAGE_MTG_I_GAIN); 
		_secondStgMtr.setD(SECOND_STAGE_MTG_D_GAIN);
		
		// Blender Motor
		_blenderMtr = new CANTalon(blenderMtrCanBusAddr);
		_blenderMtr.enableBrakeMode(false);
		_blenderMtr.enableLimitSwitch(false, false);
		
		// Feeder Motor
		_feederMtr = new CANTalon(feederMtrCanBusAddr);
		_feederMtr.enableBrakeMode(false);
		_feederMtr.enableLimitSwitch(false, false);
		
		// Slider
		_linearActuator = new Servo(sliderPWMPort);
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

	public void SpinStg1Wheel(double targetRPM)
	{
		_stg1MtrTargetRPM = targetRPM;
		
		_firstStgMtr.set(_stg1MtrTargetRPM);
		DriverStation.reportWarning("Stage 1 Target RPM = " + targetRPM, true);
	}
	
	public void SpinStg2Wheel(double targetRPM)
	{
		_stg2MtrTargetRPM = targetRPM;
		
		_secondStgMtr.set(_stg2MtrTargetRPM);
		DriverStation.reportWarning("Stage 2 Target RPM = " + targetRPM, true);
	}
	
	//============================================================================================
	// Set Up Shooter Testing
	//============================================================================================
	
	public void Stg1RPMUp()
	{
		if(_stg1MtrTargetRPM > MAX_SHOOTER_RPM)
		{
			if(_stg1MtrTargetRPM < 0)
			{
				SpinStg1Wheel(_stg1MtrTargetRPM -= 100);
			}
			else
			{
				SpinStg1Wheel(-3000);
			}
		}
	}
	public void Stg2RPMUp()
	{
		if(_stg2MtrTargetRPM > MAX_SHOOTER_RPM)
		{
			if(_stg2MtrTargetRPM < 0)
			{
				SpinStg2Wheel(_stg2MtrTargetRPM -= 100);
			}
			else
			{
				SpinStg2Wheel(-3000);
			}
		}
	}
	public void Stg1RPMDown()
	{
		if(_stg1MtrTargetRPM < MIN_SHOOTER_RPM)
		{
			SpinStg1Wheel(_stg1MtrTargetRPM += 100);
		}
	}
	public void Stg2RPMDown()
	{
		if(_stg2MtrTargetRPM < MIN_SHOOTER_RPM)
		{
			SpinStg2Wheel(_stg2MtrTargetRPM += 100);
		}
	}
	
	//============================================================================================
	// Blender/Feeder Motors
	//============================================================================================
	
	public void SpinBlender()
	{
		SpinBlender(BLENDER_PERCENTVBUS_COMMAND);
	}
	
	public void SpinFeeder()
	{
		SpinFeeder(FEEDER_PERCENTVBUS_COMMAND);
	}
	
	private void SpinBlender(double blenderVbusCommand)
	{
		_blenderMtr.set(blenderVbusCommand);
	}
	
	private void SpinFeeder(double feederVbusCommand)
	{
		_feederMtr.set(feederVbusCommand);
	}
	
	//============================================================================================
	// Linear Actuator
	//============================================================================================
	
	public void ActuatorInitialConfig()
	{
		_currentSliderPosition = INITIAL_POSITION_ACTUATOR;
		_linearActuator.setPosition(_currentSliderPosition);
		DriverStation.reportWarning("Actuator Configured to " + _currentSliderPosition, false);
	} 
	
	public void ActuatorUp()
	{
		if (_currentSliderPosition < MAX_THRESHOLD_ACTUATOR)
		{
			_currentSliderPosition += CHANGE_INTERVAL_ACTUATOR;
			_currentSliderPosition = Utilities.RoundDouble(_currentSliderPosition, 3); //rounds to 3 Decimal Places
			_linearActuator.setPosition(_currentSliderPosition);
			DriverStation.reportWarning("Actuator Position " + _currentSliderPosition, false);
		}
		else
		{
			DriverStation.reportWarning("!=!=!=!=! Actuator Already at Maximum Position", false);
		}
	}
	
	public void ActuatorDown()
	{
		if (_currentSliderPosition > MIN_THRESHOLD_ACTUATOR)
		{
			_currentSliderPosition -= CHANGE_INTERVAL_ACTUATOR;
			_currentSliderPosition = Utilities.RoundDouble(_currentSliderPosition, 3); //rounds to 3 Decimal Places
			_linearActuator.setPosition(_currentSliderPosition);
			DriverStation.reportWarning("Actuator Position " + _currentSliderPosition, false);
		}
		else
		{
			DriverStation.reportWarning("!=!=!=!=! Actuator Already at Minimum Position", false);
		}
	}
	
	//============================================================================================
	// Update Smart Dashboard with Current Values
	//============================================================================================	
	
	public void OutputToSmartDashboard()
	{
		String outDataStg1 = "?";
		String outDataStg2 = "?";
		String outDataActuator = "?";
		
		//Display Current Shooter Motor RPM + Error
		outDataStg1 = String.format( "%.0f RPM (%.2f%%)", getStg1ActualRPM(), getStg1RPMErrorPercent());
		outDataStg2 = String.format("%.0f RPM (%.2f%%)", getStg2ActualRPM(), getStg2RPMErrorPercent());
		
		SmartDashboard.putString("Current Stage 1 RPM (Error)", outDataStg1);
		SmartDashboard.putString("Current Stage 2 RPM (Error)", outDataStg2);

		
		//Display Current Actuator Value
		outDataActuator = String.format( "%.3f", _currentSliderPosition); //Outputs "Max" and "Min" at respective values
		if(_currentSliderPosition == MAX_THRESHOLD_ACTUATOR)
		{
			outDataActuator = outDataActuator + " (MAX)";
		}
		else if(_currentSliderPosition == MIN_THRESHOLD_ACTUATOR)
		{
			outDataActuator = outDataActuator + " (MIN)";
		}
		SmartDashboard.putString("Actuator Current Value", outDataActuator);
	}
	
	//============================================================================================
	// Update Logging File
	//============================================================================================	
	
	public void UpdateLogData(LogData logData)
	{
		logData.AddData("Stg1Mtr:Cmd_Rpm", String.format("%f", _stg1MtrTargetRPM));
		logData.AddData("Stg1Mtr:Act_Rpm", String.format("%f", getStg1ActualRPM()));
		logData.AddData("Stg1Mtr:Err_%", String.format("%.2f%%", getStg1RPMErrorPercent()));
		logData.AddData("Stg1Mtr: %VBus", String.format("%.2f%%", getStg1CurrentPercentVBus()));
			
		logData.AddData("Stg2Mtr:Cmd_Rpm", String.format("%f", _stg2MtrTargetRPM));
		logData.AddData("Stg2Mtr:Act_Rpm", String.format("%f", getStg2ActualRPM()));	
		logData.AddData("Stg2Mtr:Err_%", String.format("%.2f%%", getStg2RPMErrorPercent()));
		logData.AddData("Stg2Mtr: %VBus", String.format("%.2f%%", getStg2CurrentPercentVBus()));

		
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
	public double getStg1CurrentPercentVBus()
	{
		double currentOutputVoltage = _firstStgMtr.getOutputVoltage();
		double currentBusVoltage = _firstStgMtr.getBusVoltage();
		
		double currentActualSpeed = (currentOutputVoltage / currentBusVoltage);
		
		return Utilities.RoundDouble(currentActualSpeed, 2);
	}
	
	private double getStg2ActualRPM()
	{
		return _secondStgMtr.getSpeed();
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
	public double getStg2CurrentPercentVBus()
	{
		double currentOutputVoltage = _secondStgMtr.getOutputVoltage();
		double currentBusVoltage = _secondStgMtr.getBusVoltage();
		
		double currentActualSpeed = (currentOutputVoltage / currentBusVoltage);
		
		return Utilities.RoundDouble(currentActualSpeed, 2);
	}
}
