package org.usfirst.frc.team4028.robot.subsystems;


import org.usfirst.frc.team4028.robot.utilities.LogData;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	
	public void InfeedNoSolenoid()
	{
		//_fuelInfeedSolenoid.set(false);
		_fuelInfeedMtr.set(FUEL_INFEED_MOTOR_SPEED);
	}
	
	public void ToggleSolenoid()
	{
		_fuelInfeedSolenoid.set(!_fuelInfeedSolenoid.get());
		
		//if(_fuelInfeedSolenoid.get() == false)
		//{
		//	_fuelInfeedSolenoid.set(true);
		//}
		//else if(_fuelInfeedSolenoid.get() == true)
		//{
		//	_fuelInfeedSolenoid.set(false);
		//}
	}
	
	// update the Dashboard with any Climber specific data values
	public void OutputToSmartDashboard()
	{
		SmartDashboard.putBoolean("Is Fuel Infeed Tilt Extended", _fuelInfeedSolenoid.get());
		SmartDashboard.putString( "Ball Infeed Cmd", String.format("%.3f", _fuelInfeedMtr.getOutputVoltage()/_fuelInfeedMtr.getBusVoltage()));
	}
	
	public void UpdateLogData(LogData logData)
	{
	}
	
	//============================================================================================
	// Property Accessors follow
	//============================================================================================

}
