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
//	3		Patrick		2/20 18:47		Updating Values Written to SmartDashboard
//	4		Patrick		2/22 12:34		Making toggle button for Blender/Feeder
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
	
	// define class level working variables
	private double _stg1MtrTargetRPM;
	private double _stg2MtrTargetRPM;
	
	private double _currentSliderPosition;
	
	//define class level PID constants
	private static final double FIRST_STAGE_MTG_FF_GAIN = 0.0278;
	private static final double FIRST_STAGE_MTG_P_GAIN = 0.05;
	private static final double FIRST_STAGE_MTG_I_GAIN = 0.0;
	private static final double FIRST_STAGE_MTG_D_GAIN = 0.5;

	private static final double SECOND_STAGE_MTG_FF_GAIN = 0.0274;
	private static final double SECOND_STAGE_MTG_P_GAIN = 0.0;
	private static final double SECOND_STAGE_MTG_I_GAIN = 0.0;
	private static final double SECOND_STAGE_MTG_D_GAIN = 0.0; //0.115;
	
	//define class level Actuator Constants
	private static final double MAX_THRESHOLD_ACTUATOR = 0.7; 
	private static final double MIN_THRESHOLD_ACTUATOR = 0.4;
	private static final double CHANGE_INTERVAL_ACTUATOR = 0.025;
	private static final double INITIAL_POSITION_ACTUATOR = 0.55;
	
	//define class level Shooter Motor Constants
	private static final double MAX_SHOOTER_RPM = -4400;
	private static final double MIN_SHOOTER_RPM = -3000;
	private static final double SHOOTER_BUMP_RPM = 100;
	private static final double FIRST_STAGE_MTR_DEFAULT_RPM = -3700;
	private static final double SECOND_STAGE_MTR_DEFAULT_RPM = -3700;
	
	private static final double FEEDER_PERCENTVBUS_COMMAND = -0.7; //This Mo tor Needs to Run in Reverse
	private static final double BLENDER_PERCENTVBUS_COMMAND = 0.35;

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
	
	public void Stg1MtrBumpRPMUp()
	{
		// only bump if not already at max
		if(Math.abs(_stg1MtrTargetRPM) >= Math.abs(MAX_SHOOTER_RPM))
		{	
			if(Math.abs(_stg1MtrTargetRPM) >  0)
			{
				// if already turning, just bump
				SpinStg1Wheel(_stg1MtrTargetRPM -= SHOOTER_BUMP_RPM);
			}
			else
			{
				// if currently stopped, set to default speed
				SpinStg1Wheel(FIRST_STAGE_MTR_DEFAULT_RPM);
			}
		}
		else
		{
			DriverStation.reportWarning("Stg 1 Mtr Already at MAX ", false);
		}
	}
	
	public void Stg1MtrBumpRPMDown()
	{
		// only bump if not already at min
		if(Math.abs(_stg1MtrTargetRPM) < Math.abs(MIN_SHOOTER_RPM))
		{
			SpinStg1Wheel(_stg1MtrTargetRPM += SHOOTER_BUMP_RPM);
		}
		else
		{
			DriverStation.reportWarning("Stg 1 Mtr Already at MIN ", false);
		}
	}
	
	public void Stg2MtrBumpRPMUp()
	{
		// only bump if not already at max
		if(Math.abs(_stg2MtrTargetRPM) >= Math.abs(MAX_SHOOTER_RPM))
		{	
			if(Math.abs(_stg2MtrTargetRPM) >  0)
			{
				// if already turning, just bump
				SpinStg1Wheel(_stg2MtrTargetRPM -= SHOOTER_BUMP_RPM);
			}
			else
			{
				// if currently stopped, set to default speed
				SpinStg2Wheel(SECOND_STAGE_MTR_DEFAULT_RPM);
			}
		}
		else
		{
			DriverStation.reportWarning("Stg 2 Mtr Already at MAX ", false);
		}
	}

	public void Stg2MtrBumpRPMDown()
	{
		// only bump if not already at min
		if(Math.abs(_stg2MtrTargetRPM) < Math.abs(MIN_SHOOTER_RPM))
		{
			SpinStg2Wheel(_stg2MtrTargetRPM += SHOOTER_BUMP_RPM);
		}
		else
		{
			DriverStation.reportWarning("Stg 2 Mtr Already at MIN ", false);
		}
	}
	
	//============================================================================================
	// Blender/Feeder Motors
	//============================================================================================
	
	public void ToggleSpinBlender()
	{
		// if current cmd is 0, then start
		if(_blenderMtr.get() == 0)
		{
			SpinBlender(BLENDER_PERCENTVBUS_COMMAND);
		}
		else
		{
			SpinBlender(0.0);
		}
	}
	
	public void ToggleSpinFeeder()
	{
		// if current cmd is 0, then start
		if(_feederMtr.get() == 0)
		{
			SpinFeeder(FEEDER_PERCENTVBUS_COMMAND);
		}
		else
		{
			SpinFeeder(0.0);
		}
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
	
	public void ActuatorMoveToDefaultPosition()
	{
		_currentSliderPosition = INITIAL_POSITION_ACTUATOR;
		_linearActuator.setPosition(_currentSliderPosition);
		
		DriverStation.reportWarning("Actuator Configured to " + _currentSliderPosition, false);
	} 
	
	public void ActuatorMoveUp()
	{
		if (_currentSliderPosition < MAX_THRESHOLD_ACTUATOR)
		{
			_currentSliderPosition += CHANGE_INTERVAL_ACTUATOR;
			
			_currentSliderPosition = Utilities.RoundDouble(_currentSliderPosition, 3); //rounds to 3 Decimal Places
			_linearActuator.setPosition(_currentSliderPosition);
			
			DriverStation.reportWarning("Actuator Move Up To: " + _currentSliderPosition, false);
		}
		else
		{
			DriverStation.reportWarning("!=!=!=!=! Actuator Already at MAXIMUM Position", false);
		}
	}
	
	public void ActuatorMoveDown()
	{
		if (_currentSliderPosition > MIN_THRESHOLD_ACTUATOR)
		{
			_currentSliderPosition -= CHANGE_INTERVAL_ACTUATOR;
			
			_currentSliderPosition = Utilities.RoundDouble(_currentSliderPosition, 3); //rounds to 3 Decimal Places
			_linearActuator.setPosition(_currentSliderPosition);
			
			DriverStation.reportWarning("Actuator Move Down To: " + _currentSliderPosition, false);
		}
		else
		{
			DriverStation.reportWarning("!=!=!=!=! Actuator Already at MINIMUM Position", false);
		}
	}
	
	//============================================================================================
	// Update Smart Dashboard with Current Values
	//============================================================================================	
	
	public void OutputToSmartDashboard()
	{
		// working varibles
		String outDataStg1Actual = "?";
		String outDataStg2Actual = "?";
		//String outDataStg1Command = "?";
		//String outDataStg2Command = "?";
		String outDataActuator = "?";
		
		//Display Current Shooter Motor 1 & 2 Cmd & Actual RPM + Error
		outDataStg1Actual = String.format("[%.0f RPM] %.0f RPM (%.2f%%)", -1*_stg1MtrTargetRPM, -1*getStg1ActualRPM(), getStg1RPMErrorPercent());
		outDataStg2Actual = String.format("[%.0f RPM] %.0f RPM (%.2f%%)", -1*_stg1MtrTargetRPM, -1*getStg2ActualRPM(), getStg2RPMErrorPercent());
		//outDataStg1Command = String.format("%.0f RPM", _stg1MtrTargetRPM);
		//outDataStg2Command = String.format("%.0f RPM", _stg2MtrTargetRPM);
		
		SmartDashboard.putString("[Command] Current Stage 1 RPM (Error)", outDataStg1Actual);
		SmartDashboard.putString("[Command] Current Stage 2 RPM (Error)", outDataStg2Actual);
		//SmartDashboard.putString("Current Stage 1 Command RPM", outDataStg1Command);
		//SmartDashboard.putString("Current Stage 2 Command RPM", outDataStg2Command);

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
		logData.AddData("Stg1Mtr:%VBus", String.format("%.2f%%", getStg1CurrentPercentVBus()));
			
		logData.AddData("Stg2Mtr:Cmd_Rpm", String.format("%f", _stg2MtrTargetRPM));
		logData.AddData("Stg2Mtr:Act_Rpm", String.format("%f", getStg2ActualRPM()));	
		logData.AddData("Stg2Mtr:Err_%", String.format("%.2f%%", getStg2RPMErrorPercent()));
		logData.AddData("Stg2Mtr:%VBus", String.format("%.2f%%", getStg2CurrentPercentVBus()));

		
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
