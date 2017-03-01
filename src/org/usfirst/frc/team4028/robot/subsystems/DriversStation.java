package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.LogData;

import edu.wpi.first.wpilibj.DriverStation;

//This class implements all functionality to read Operator & Driver Gamepads
//
//------------------------------------------------------
//	Rev		By		 	D/T				Desc
//	===		========	===========		=================================
//	0		TomB		17.Feb.2017		Initial Version
//	1		TomB		18.FEb.2018		added OPERATOR_START_BUTTON			Gear Tilt ReZero
//------------------------------------------------------
//
public class DriversStation extends BaseDriversStation
{
	//============================================================================================
	// constructors follow
	//============================================================================================
	public DriversStation(int driverGamePadUsbPort, int operatorGamePadUsbPort)
	{
		// call base class constructor
		super(driverGamePadUsbPort, operatorGamePadUsbPort);
	}
	
	/*
	==========================================================================
	--- Driver Joysticks --------
	DRIVER_LEFT_X_AXIS
	DRIVER_LEFT_Y_AXIS				ChassisThrottle_JoystickCmd
	DRIVER_LEFT_TRIGGER
	DRIVER_RIGHT_TRIGGER
	DRIVER_RIGHT_X_AXIS				ChassisTurn_JoystickCmd
	DRIVER_RIGHT_Y_AXIS
	
	--- Driver Buttons --------
	DRIVER_GREEN_BUTTON_A			Feeder And Blender (AccDecModeToggle_Btn)
	DRIVER_RED_BUTTON_B				Shooter Slider Down
	DRIVER_BLUE_BUTTON_X			GearShiftToggle_Btn
	DRIVER_YELLOW_BUTTON_Y			Shooter Slider Up
	DRIVER_LEFT_BUMPER				Shooter Stage 1 Motor RPM Up 100
	DRIVER_RIGHT_BUMPER				Shooter Stage 2 Motor RPM Up 100
	DRIVER_BACK_BUTTON				Shooter Stage 1 Motor RPM Down 100
	DRIVER_START_BUTTON				Shooter Stage 2 Motor RPM Down 100
	DRIVER_LEFT_THUMBSTICK		
	DRIVER_RIGHT_THUMBSTICK
		
	==========================================================================
		
	--- Operator Joysticks --------
	OPERATOR_LEFT_X_AXIS
	OPERATOR_LEFT_Y_AXIS			
	OPERATOR_LEFT_TRIGGER
	OPERATOR_RIGHT_TRIGGER
	OPERATOR_RIGHT_X_AXIS
	OPERATOR_RIGHT_Y_AXIS			GearInfeedOutFeed_JoystickCmd
	
	--- Operator Buttons --------
	OPERATOR_GREEN_BUTTON_A			Gear Tilt Floor
	OPERATOR_RED_BUTTON_B			Gear Tilt Score 
	OPERATOR_BLUE_BUTTON_X			Gear Sequence Initiation
	OPERATOR_YELLOW_BUTTON_Y		Gear Tilt Home
	OPERATOR_LEFT_BUMPER			Fuel Infeed Motor
	OPERATOR_RIGHT_BUMPER			Camera Swap
	OPERATOR_BACK_BUTTON			Ball Infeed Tilt Toggle
	OPERATOR_START_BUTTON			Gear Tilt ReZero
	OPERATOR_LEFT_THUMBSTICK
	OPERATOR_RIGHT_THUMBSTICK
	==========================================================================
	*/
		
	// ======================================================
	// Public Property Accessors
	//		Implement calls into base class property accessors with 
	//		Robot function specific names
	// ======================================================
	
	// ===================================
	// === driver Just Pressed buttons ===
	// ===================================
	/*   <sample code>
	public boolean getIsDriver_FEATUREA_BtnJustPressed()
	{
		return super.getIsDriverGreenBtnAJustPressed();
	} 
	*/
	
	// ConstantVelocityThroughVBus
	public boolean getIsDriver_GearShiftToggle_BtnJustPressed()
	{
		return super.getIsDriverBlueBtnXJustPressed();
	}
	
	// AccDec Mode
	public boolean getIsDriver_ToggleBlenderAndFeederMtrs_BtnJustPressed()
	{
		return super.getIsDriverGreenBtnAJustPressed();
	}
	
	//ShooterStg1Up
	//public boolean getIsDriver_ShooterStg1Up_BtnJustPressed()
	//{
	//	return super.getIsDriverLeftBumperBtnJustPressed();
	//}
	
	//ShooterStg1Down
	//public boolean getIsDriver_ShooterStg1Down_BtnJustPressed()
	//{
	//	return super.getIsDriverBackBtnJustPressed();
	//}
	
	// Shooter Stg 1 Cycle Up/Down
	public boolean getIsDriver_ShooterStg1CycleRPM_BtnJustPressed()
	{
		return super.getIsDriverLeftBumperBtnJustPressed();
	}
	
	// Blender Cycle
	public boolean getIsDriver_BlenderCycle_BtnJustPressed()
	{
		return super.getIsDriverStartBtnJustPressed();
	}
	
	//ShooterStg2Down
	//public boolean getIsDriver_ShooterStg2Down_BtnJustPressed()
	//{
	//	return super.getIsDriverStartBtnJustPressed();
	//}
	
	// Shooter Stg 2 Cycle Up/Down
	public boolean getIsDriver_ShooterStg2CycleRPM_BtnJustPressed()
	{
		return super.getIsDriverRightBumperBtnJustPressed();
	}
	
	//ActuatorUp
	public boolean getIsDriver_ActuatorUp_BtnJustPressed()
	{
		return super.getIsDriverYellowBtnYJustPressed();
	}
	
	//ActuatorDown
	public boolean getIsDriver_ActuatorDown_BtnJustPressed()
	{
		return super.getIsDriverRedBtnBJustPressed();
	}
	
	//FullStop
	public boolean getIsDriver_FullShooterStop_BtnJustPressed() //HERE
	{
		return super.getIsDriverBackBtnJustPressed();
	}
		
	// ===================================
	// === driver Is Pressed buttons =====
	// ===================================
	/*  <sample code>
	public boolean getIsDriver_FEATUREA_BtnAPressed()
	{
		return super.getIsDriverGreenBtnAPressed();
	}
	*/
	
	// ===================================
	// === driver Joysticks ==============
	// ===================================
	/*	<sample code>
	public double getDriver_FEATUREB_JoystickCmd()
	{
		return super.getDriverLeftXAxisCmd();
	}
	*/
	
	// Chassis Throttle
	public double getDriver_ChassisThrottle_JoystickCmd()
	{
		return super.getDriverLeftYAxisCmd();
	}
	
	// Chassis Turn
	public double getDriver_ChassisTurn_JoystickCmd()
	{
		return super.getDriverRightXAxisCmd();
	}
	
	// =====================================
	// === operator Just Pressed buttons ===
	// =====================================
	
	// Gear AutoScore Sequence
	public boolean getIsOperator_GearStartSequence_BtnJustPressed()
	{
		return super.getIsOperatorBlueBtnXJustPressed();
	}
	
	// Gear Tilt Home
	public boolean getIsOperator_GearGoToHome_BtnJustPressed()
	{
		return super.getIsOperatorYellowBtnYJustPressed();
	}
	
	// Gear ReZero
	public boolean getIsOperator_GearReZero_BtnJustPressed()
	{
		return super.getIsOperatorStartBtnJustPressed();
	}
	
	// Gear Tilt Floor
	public boolean getIsOperator_GearGoToFloor_BtnJustPressed()
	{
		return super.getIsOperatorGreenBtnAJustPressed();
	}
	
	// Gear Tilt Score 
	public boolean getIsOperator_GearGoToScore_BtnJustPressed()
	{
		return super.getIsOperatorRedBtnBJustPressed();
	}
	
	// Swap Cameras
	public boolean getIsOperator_CameraSwap_BtnJustPressed()
	{
		return super.getIsOperatorRightBumperBtnJustPressed();
	}
	
	public boolean getIsOperator_StartClimb_ButtonJustPressed()
	{
		return super.getIsOperatorBackBtnJustPressed();
	}
	
	// =====================================
	// === operator Is Pressed buttons =====
	// =====================================
	
	//Fuel Infeed
	public boolean getIsOperator_FuelInfeed_BtnPressed()
	{
		return super.getIsOperatorLeftBumperBtnPressed();
	}
	
	/*
	// Gear Infeed
	public boolean getIsOperator_GearInfeed_BtnPressed()
	{
		return super.getIsOperatorGreenBtnAPressed();
	}	
	
	// Gear Outfeed
	public boolean getIsOperator_GearOutfeed_BtnPressed()
	{
		return super.getIsOperatorRedBtnBPressed();
	}
	*/
	
	// =====================================
	// === operator Joysticks ==============
	// =====================================
	
	// Winch
	public double getOperator_Winch_JoystickCmd()
	{
		//return super.getOperatorRightYAxisCmd();
		return 0;
	}
	
	
	// GearInfeedOutFeed
	public double getOperator_GearInfeedOutFeed_JoystickCmd()
	{
		return super.getOperatorRightYAxisCmd();
	}
	
	public double getOperator_GearTiltFeed_JoystickCmd()
	{
		return super.getOperatorLeftYAxisCmd();
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	
	// update the Dashboard with any Drivers Station specific data values
	public void OutputToSmartDashboard()
	{
		
	}
	
	public void UpdateLogData(LogData logData)
	{
		
	}
}

