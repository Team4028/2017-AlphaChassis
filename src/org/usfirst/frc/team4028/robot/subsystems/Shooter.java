package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.LogData;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.PWM;

//This class implements all functionality for the SHOOTER (& Blender) Subsystem
//=====> For Changes see TBD
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
	
	// define class level variables for Robot objects
	private CANTalon _firstStgMtr;
	private CANTalon _secondStageMtr;
	
	private PWM _linearactuator;
	
	// define class level working varibles
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
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public Shooter(int firstStgMtrCanBusAddr, int secondStageMtrCanBusAddr,
					int sliderPWMPort)
	{
		// First Stage Motor
		_firstStgMtr = new CANTalon(firstStgMtrCanBusAddr);
		_firstStgMtr.changeControlMode(CANTalon.TalonControlMode.Speed);	// open loop throttle
		_firstStgMtr.enableBrakeMode(false);							// default to brake mode DISABLED
    	_firstStgMtr.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);	// set encoder to be feedback device
    	_firstStgMtr.reverseSensor(true);  							// do not invert encoder feedback
		_firstStgMtr.enableLimitSwitch(false, false);
        /* set the peak and nominal outputs, 12V means full */
		_firstStgMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		_firstStgMtr.configPeakOutputVoltage(12.0f, 0.0f);
    	
		/* set closed loop gains in slot0 */
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
        /* set the peak and nominal outputs, 12V means full */
		_secondStageMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		_secondStageMtr.configPeakOutputVoltage(12.0f, 0.0f);
		
		/* set closed loop gains in slot0 */
		_secondStageMtr.setProfile(0);
		_secondStageMtr.setF(SECOND_STAGE_MTG_FF_GAIN); 
		_secondStageMtr.setP(SECOND_STAGE_MTG_P_GAIN); 
		_secondStageMtr.setI(SECOND_STAGE_MTG_I_GAIN); 
		_secondStageMtr.setD(SECOND_STAGE_MTG_D_GAIN);
		
		// Slider
		_linearactuator = new PWM(sliderPWMPort);
	}

    /* set closed loop gains in slot0 */
    //_talon.setProfile(0);
    //_talon.setF(0.035); //(0.02854); 
    //_talon.setP(0.075); //(0.11333); //(.2046); 
    //_talon.setI(0); 
    //_talon.setD(0);
    
    /*  M1
    //_talon.setProfile(0);
    //_talon.setF(0.0290); //(0.02854); 
    //_talon.setP(0.075); //(0.11333); //(.2046); 
    //_talon.setI(0); 
    //_talon.setD(0);
    */
	
	//============================================================================================
	// Methods follow
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
	
	public void FullStop() 
	{
		SpinStg1Wheel(0);
		SpinStg2Wheel(0);
	}
	
	// update the Dashboard with any Climber specific data values
	public void OutputToSmartDashboard()
	{
	}
	
	// update the log data
	public void UpdateLogData(LogData logData)
	{
		logData.AddData("Stg1Mtr:Cmd_Rpm", String.format("%d", _stg1MtrTargetRPM));
		logData.AddData("Stg1Mtr:Act_Rpm", String.format("%d", getStg1ActualRPM()));
		logData.AddData("Stg1Mtr:Err_%", String.format("%.2f%%", getStg1RPMErrorPercent()));
		
		logData.AddData("Stg2Mtr:Cmd_Rpm", String.format("%d", _stg2MtrTargetRPM));
		logData.AddData("Stg2Mtr:Act_Rpm", String.format("%d", getStg2ActualRPM()));	
		logData.AddData("Stg2Mtr:Err_%", String.format("%.2f%%", getStg2RPMErrorPercent()));
	}
	
	//============================================================================================
	// Property Accessors follow
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
