package org.usfirst.frc.team4028.robot.subsystems;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.ALLIANCE;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.AUTON_MODE;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//This class implements all functionality to read settings from the Dashboard
//
//------------------------------------------------------
//	Rev		By		 	D/T			Desc
//	===		========	===========	=================================
//	0		Sydney	 	15.Feb.2017	Initial Version
//	1		TomB		26.Feb.2017	Updated w/ new Auton Options
//------------------------------------------------------
//
//=====> For Changes see Sydney
public class DashboardInputs
{ 
	
	private AUTON_MODE _autonModeChoice;
	private ALLIANCE _alliance;
	
	private SendableChooser<AUTON_MODE> _autonModeChooser;
	private SendableChooser<ALLIANCE> _allianceChooser;
	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public DashboardInputs()
	{
		ConfigAutonModeChoosers();
	}
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	private void ConfigAutonModeChoosers()
	{
		//============================
		// Autonomous Mode Choice
		//============================
		_autonModeChooser = new SendableChooser<AUTON_MODE>();
		
		//CROSS_BASE_LINE,
		//DO_NOTHING,
		//HANG_BOILER_SIDE_GEAR,
		//HANG_CENTER_GEAR,
		//HANG_KEY_SIDE_GEAR,
		//TURN_AND_SHOOT
		
		_autonModeChooser.addObject("Do Nothing", GeneralEnums.AUTON_MODE.DO_NOTHING);
		_autonModeChooser.addObject("Cross the Base Line", GeneralEnums.AUTON_MODE.CROSS_BASE_LINE);
		_autonModeChooser.addObject("Hang Gear on Boiler Side", GeneralEnums.AUTON_MODE.HANG_BOILER_SIDE_GEAR);
		_autonModeChooser.addDefault("Hang Gear in Center", GeneralEnums.AUTON_MODE.HANG_CENTER_GEAR);
		_autonModeChooser.addObject("Hang Gear on Key Side", GeneralEnums.AUTON_MODE.HANG_KEY_SIDE_GEAR);
		_autonModeChooser.addObject("Turn and Shoot", GeneralEnums.AUTON_MODE.TURN_AND_SHOOT);
		
		SmartDashboard.putData("Auton Mode Chooser", _autonModeChooser);
		_autonModeChoice = _autonModeChooser.getSelected();
		
		//============================
		// Alliance Choice
		//============================
		_allianceChooser = new SendableChooser<ALLIANCE>();
		
		_allianceChooser.addDefault("Red Alliance", GeneralEnums.ALLIANCE.RED_ALLIANCE);
		_allianceChooser.addObject("Blue Alliance", GeneralEnums.ALLIANCE.BLUE_ALLIANCE);
		
		SmartDashboard.putData("Alliance Chooser" , _allianceChooser);
		_alliance =  _allianceChooser.getSelected();
		
	}

	//============================================================================================
	// Property Accessors follow
	//============================================================================================
	public AUTON_MODE get_autonMode()
	{
		return _autonModeChoice;
	}
	
	public ALLIANCE get_allianceMode()
	{
		return _alliance;
	}
}
