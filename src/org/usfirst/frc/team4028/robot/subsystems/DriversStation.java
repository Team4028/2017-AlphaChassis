package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.utilities.LogData;

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
	DRIVER_LEFT_TRIGGER				Shooter Stg1 Cycle Down RPM
	DRIVER_RIGHT_TRIGGER			Shooter Stg2 Cycle Down RPM
	DRIVER_RIGHT_X_AXIS				ChassisTurn_JoystickCmd
	DRIVER_RIGHT_Y_AXIS
	
	--- Driver Buttons --------
	DRIVER_GREEN_BUTTON_A			Toggle Blender And Feeder Mtrs
	DRIVER_RED_BUTTON_B				Shooter Slider Down
	DRIVER_BLUE_BUTTON_X			Gear Shift Toggle
	DRIVER_YELLOW_BUTTON_Y			Shooter Slider Up
	DRIVER_LEFT_BUMPER				Shooter Stg1 Cycle RPM
	DRIVER_RIGHT_BUMPER				Shooter Stg2 Cycle RPM
	DRIVER_BACK_BUTTON				Full Shooter Stop
	DRIVER_START_BUTTON				Blender Cycle RPM
	DRIVER_LEFT_THUMBSTICK		
	DRIVER_RIGHT_THUMBSTICK
		
									Acc/Dec Mode Toggle
									Shooter Stg 1 Motor RPM + 100
									Shooter Stg 1 Motor RPM - 100
									Shooter Stg 2 Motor RPM + 100
									Shooter Stg 2 Motor RPM - 100
	==========================================================================
		
	--- Operator Joysticks --------
	OPERATOR_LEFT_X_AXIS
	OPERATOR_LEFT_Y_AXIS			Gear Tilt
	OPERATOR_LEFT_TRIGGER
	OPERATOR_RIGHT_TRIGGER
	OPERATOR_RIGHT_X_AXIS
	OPERATOR_RIGHT_Y_AXIS			Gear Infeed/OutFeed
	
	--- Operator Buttons --------
	OPERATOR_GREEN_BUTTON_A			Gear Tilt Goto Floor
	OPERATOR_RED_BUTTON_B			Gear Tilt Goto Score 
	OPERATOR_BLUE_BUTTON_X			Hang Gear Sequence Initiation
	OPERATOR_YELLOW_BUTTON_Y		Gear Tilt GoTo Home
	OPERATOR_LEFT_BUMPER			Fuel Infeed
	OPERATOR_RIGHT_BUMPER			Camera Swap
	OPERATOR_BACK_BUTTON			Start Climb
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
	
	// =========================================================================================================
	// DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER	DRIVER
	// =========================================================================================================
	
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
	//public boolean getIsDriver_ShooterStg1CycleRPM_BtnJustPressed()
	//{
	//	return super.getIsDriverLeftBumperBtnJustPressed();
	//}
	
	public boolean getIsDriver_ShooterStg1StepRPMUp_BtnJustPressed()
	{
		return super.getIsDriverLeftBumperBtnJustPressed();
	}
	
	// Blender Cycle
	public boolean getIsDriver_BlenderCycleRPM_BtnJustPressed()
	{
		return super.getIsDriverStartBtnJustPressed();
	}
	
	//ShooterStg2Down
	//public boolean getIsDriver_ShooterStg2Down_BtnJustPressed()
	//{
	//	return super.getIsDriverStartBtnJustPressed();
	//}
	
	// Shooter Stg 2 Cycle Up/Down
	//public boolean getIsDriver_ShooterStg2CycleRPM_BtnJustPressed()
	//{
	//	return super.getIsDriverRightBumperBtnJustPressed();
	//}
	
	public boolean getIsDriver_ShooterStg2StepRPMUp_BtnJustPressed()
	{
		return super.getIsDriverRightBumperBtnJustPressed();
	}
	
	//ActuatorUp
	public boolean getIsDriver_ShooterSliderUp_BtnJustPressed()
	{
		return super.getIsDriverYellowBtnYJustPressed();
	}
	
	//ActuatorDown
	public boolean getIsDriver_ShooterSliderDown_BtnJustPressed()
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
	
	
	// Stg 1 Mtr Down
	public boolean getIsDriver_ShooterStg1StepRPMDown_BtnJustPressed()
	{
		return (Math.abs(super.getDriverLeftTriggerCmd()) > 0.2);
	}
	
	// Stg 2 Mtr Down
	public boolean getIsDriver_ShooterStg2StepRPMDown_BtnJustPressed()
	{
		return (Math.abs(super.getDriverRightTriggerCmd()) > 0.2);
	}
	
	
	// =========================================================================================================
	// OPERATOR		OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	OPERATOR	
	// =========================================================================================================
	
	
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

