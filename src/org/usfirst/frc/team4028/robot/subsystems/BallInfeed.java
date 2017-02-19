package org.usfirst.frc.team4028.robot.subsystems;

import java.io.WriteAbortedException;

import org.usfirst.frc.team4028.robot.LogData;
import org.usfirst.frc.team4028.robot.constants.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.Solenoid;

//This class implements all functionality for the Infeed Subsystem
//
//------------------------------------------------------
//	Rev		By		 	D/T			Desc
//	===		========	===========	=================================
//------------------------------------------------------
//
//=====> For Changes see TBD
public class BallInfeed 
{
	// =====================================================================
	// 1 DC Motor
	//		1 Talon w/o Encoder				Ball Infeed
	//
	// 1 Solenoid
	// 		1 Single Action/Spring Return 	Tilt
	// =====================================================================
	
	CANTalon _fuelInfeedMtr;
	Solenoid _fuelInfeedSolenoid;
	
	//======================================
	//define class level constants
	//=======================================
	
	private static final double FUEL_INFEED_MOTOR_SPEED = 1.0;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public BallInfeed(int fuelInfeedMtrCanBusAddr, int PCMCanAddr, int fuelInfeedSolenoidPort)
	{
		_fuelInfeedMtr = new CANTalon(fuelInfeedMtrCanBusAddr);
		_fuelInfeedMtr.changeControlMode(CANTalon.TalonControlMode.PercentVbus);	// open loop throttle
		_fuelInfeedMtr.enableBrakeMode(false);							// default to brake mode DISABLED
		_fuelInfeedMtr.enableLimitSwitch(false, false);					//no limit switches
		
		_fuelInfeedSolenoid = new Solenoid(PCMCanAddr, fuelInfeedSolenoidPort);
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================	

	public void FullStop() 
	{
		_fuelInfeedSolenoid.set(false);				//retract Solenoid
		_fuelInfeedMtr.set(0);						//stop motors
		
	}
	
	public void InfeedFuelAndExtendSolenoid()
	{
		_fuelInfeedSolenoid.set(true);				//engage tilt
		_fuelInfeedMtr.set(FUEL_INFEED_MOTOR_SPEED);
	}
	
	// update the Dashboard with any Climber specific data values
	public void OutputToSmartDashboard()
	{
	}
	
	public void UpdateLogData(LogData logData)
	{
	}
	
	//============================================================================================
	// Property Accessors follow
	//============================================================================================

}
