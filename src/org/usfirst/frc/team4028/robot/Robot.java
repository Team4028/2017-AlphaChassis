package org.usfirst.frc.team4028.robot;

import org.usfirst.frc.team4028.robot.autonRoutines.CrossBaseLine;
import org.usfirst.frc.team4028.robot.autonRoutines.DoNothing;
import org.usfirst.frc.team4028.robot.autonRoutines.HangBoilerSideGear;
import org.usfirst.frc.team4028.robot.autonRoutines.HangCenterGear;
import org.usfirst.frc.team4028.robot.autonRoutines.HangKeySideGear;
import org.usfirst.frc.team4028.robot.autonRoutines.TurnAndShoot;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.AUTON_MODE;
import org.usfirst.frc.team4028.robot.constants.GeneralEnums.TELEOP_MODE;
import org.usfirst.frc.team4028.robot.controllers.HangGearController;
import org.usfirst.frc.team4028.robot.constants.RobotMap;
import org.usfirst.frc.team4028.robot.sensors.Lidar;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.sensors.SwitchableCameraServer;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;
import org.usfirst.frc.team4028.robot.utilities.DataLogger;
import org.usfirst.frc.team4028.robot.utilities.LogData;
import org.usfirst.frc.team4028.robot.utilities.GeneralUtilities;
import org.usfirst.frc.team4028.robot.subsystems.Climber;
import org.usfirst.frc.team4028.robot.subsystems.DashboardInputs;
import org.usfirst.frc.team4028.robot.subsystems.DriversStation;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.BallInfeed;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.SerialPort;

/**
 * The is the main code for:
 * 	    Team:	4028 "The Beak Squad"
 * 		Season: FRC 2017 "First Steamworks"
 * 		Robot:	Alpha Chassis
 */
public class Robot extends IterativeRobot 
{
	// this value is printed on the Driver's Station message window on startup
	private static final String ROBOT_NAME = "ALPHA Chassis";
	
	// ===========================================================
	//   Define class level instance variables for Robot Runtime objects  
	// ===========================================================
	private Chassis _chassis;
	
	private Climber _climber;
	private BallInfeed _ballInfeed;
	private GearHandler _gearHandler;
	
	private Shooter _shooter;
	
	private DashboardInputs _dashboardInputs;
	private DriversStation _driversStation;
	
	// sensors
	private Lidar _lidar;
	private NavXGyro _navX;
	private SwitchableCameraServer _switchableCameraServer;
	
	// Wrapper around data logging (will be null if logging is not enabled)
	private DataLogger _dataLogger;
	
	// ===========================================================
	//   Define class level instance variables for Robot State
	// ===========================================================
	private TELEOP_MODE _telopMode;
	private AUTON_MODE _autonMode;
	
	// ===========================================================
	//   Define class level instance variables for Robot Telep Sequences 
	// ===========================================================
	private HangGearController _hangGearController;
	
	// ===========================================================
	//   Define class level instance variables for Robot Auton Routines 
	// ===========================================================
	CrossBaseLine _crossBaseLineAuton;
	DoNothing _doNothingAuton;
	HangBoilerSideGear _hangBoilerSideGearAuton;
	HangCenterGear _hangCenterGearAuton;
	HangKeySideGear _hangKeySideGear;
	TurnAndShoot _turnAndShoot;
	
	// ----------------------------------------------------------------------
	// Code executed 1x at robot startup
	// ----------------------------------------------------------------------
	@Override
	public void robotInit() 
	{
        //===================
    	// write jar (build) date & time to the dashboard
        //===================
    	GeneralUtilities.WriteBuildInfoToDashboard(ROBOT_NAME);
    	
        //===================
    	// create instances (and configure) all of all robot subsystems & sensors
        //===================
		_ballInfeed = new BallInfeed(RobotMap.FUEL_INFEED_MTR_CAN_BUS_ADDR, 
										RobotMap.PCM_CAN_BUS_ADDR, 
										RobotMap.BALL_INFEED_TILT_EXTEND_PCM_PORT);
		
		_chassis = new Chassis(RobotMap.LEFT_MASTER_CAN_BUS_ADDR, 
								RobotMap.LEFT_SLAVE1_CAN_BUS_ADDR, 
								RobotMap.RIGHT_MASTER_CAN_BUS_ADDR, 
								RobotMap.RIGHT_SLAVE1_CAN_BUS_ADDR,
								RobotMap.PCM_CAN_BUS_ADDR,
								RobotMap.SHIFTER_SOLENOID_EXTEND_PCM_PORT,
								RobotMap.SHIFTER_SOLENOID_RETRACT_PCM_PORT);
		
		_climber = new Climber(RobotMap.CLIMBER_CAN_BUS_ADDR);
		
		_dashboardInputs = new DashboardInputs();
		
		_driversStation = new DriversStation(RobotMap.DRIVER_GAMEPAD_USB_PORT, 
												RobotMap.OPERATOR_GAMEPAD_USB_PORT);
	
		_gearHandler = new GearHandler(RobotMap.GEAR_TILT_CAN_BUS_ADDR, 
										RobotMap.GEAR_INFEED_CAN_BUS_ADDR);
		
		_shooter = new Shooter(RobotMap.SHOOTER_STG1_CAN_BUS_ADDR, 
								RobotMap.SHOOTER_STG2_CAN_BUS_ADDR,
								RobotMap.BLENDER_CAN_BUS_ADDR,
								RobotMap.FEEDER_CAN_BUS_ADDR,
								RobotMap.SHOOTER_SLIDER_PWM_PORT);
		
		// sensors follow
		_lidar = new Lidar(SerialPort.Port.kMXP);
		//_navX = new NavXGyro(RobotMap.NAVX_PORT);		//TODO: reenable this
		_switchableCameraServer = new SwitchableCameraServer(RobotMap.GEAR_CAMERA_NAME);
		
		// telop Controller follow
		_hangGearController = new HangGearController(_gearHandler, _chassis);
				
		//Update Dashboard Fields (push all fields to dashboard)
		OutputAllToSmartDashboard();
	}
	
	// ----------------------------------------------------------------------
	// called each time the robot enters disabled mode from either telop or auton mode
	// ----------------------------------------------------------------------
	@Override
	public void disabledPeriodic() 
	{
		// if logging was enabled, make sure we close the file
    	if(_dataLogger != null)
    	{
	    	_dataLogger.close();
	    	_dataLogger = null;
    	}
    	
    	// stop lidar polling
    	if(_lidar != null)						{ _lidar.stop(); }
    	
    	// cleanup auton resources, since they are not needed in Telop Mode
    	if(_crossBaseLineAuton != null)			{ _crossBaseLineAuton = null; }
    	
		if(_doNothingAuton != null)				{ _doNothingAuton = null; }
		
		if(_hangBoilerSideGearAuton != null)	{ _hangBoilerSideGearAuton = null; }
		
		if(_hangCenterGearAuton != null)		{ _hangCenterGearAuton = null; }
		
		if(_hangKeySideGear != null)			{ _hangKeySideGear = null; }
		
		if(_turnAndShoot != null)				{ _turnAndShoot = null; }
	}
		
	// ----------------------------------------------------------------------
	// code executed 1x when entering AUTON Mode
	// ----------------------------------------------------------------------
	@Override
	public void autonomousInit() 
	{
		// =====================================
    	// Step 1: pre-state cleanup
		// =====================================
		
		// stop gear
    	_gearHandler.FullStop();
    	
    	// start the lidar polling
    	//if(_lidar != null)	{ _lidar.start(); }	//TODO: reslove timeout
    	
    	// =====================================
		// Step 2: add logic to read from Dashboard Choosers to select the Auton routine to run
    	// =====================================
    	// TODO: add this code

    	
    	// =====================================
		// Step 2.1: Create the correct autom routine
    	//				since we have quite a few auton routines we only create the one we need
    	//				NOTE: each "real" auton routine is responsible to call the ZeroGearTiltAxisReentrant
    	//						as many times as required
    	// =====================================
    	switch (_autonMode)
    	{
			case CROSS_BASE_LINE:
				_crossBaseLineAuton = new CrossBaseLine(_gearHandler, _chassis);
				_crossBaseLineAuton.Initialize();
				break;
				
			case DO_NOTHING:
				_doNothingAuton = new DoNothing();
				_doNothingAuton.Initialize();
				break;
				
			case HANG_BOILER_SIDE_GEAR:
				_hangBoilerSideGearAuton = new HangBoilerSideGear(_gearHandler, _chassis);
				_hangBoilerSideGearAuton.Initialize();
				break;
				
			case HANG_CENTER_GEAR:
				_hangCenterGearAuton = new HangCenterGear(_gearHandler, _chassis);
				_hangCenterGearAuton.Initialize();
				break;
				
			case HANG_KEY_SIDE_GEAR:
				_hangKeySideGear = new HangKeySideGear(_gearHandler, _chassis);
				_hangKeySideGear.Initialize();
				break;
				
			case TURN_AND_SHOOT:
				_turnAndShoot = new TurnAndShoot(_gearHandler, _chassis, _shooter);
				_turnAndShoot.Initialize();
				break;
				
			case UNDEFINED:
			default:
				DriverStation.reportError("Error... NO AUTON Selected!", false);
				break;
    	}
		
    	// =====================================
    	// Step 3: Optionally Configure Logging
    	// =====================================
    	_dataLogger = GeneralUtilities.setupLogging("auton");
	}

	// ----------------------------------------------------------------------
	// Code executed every scan cycle (about every 20 mSec or 50x / sec) in AUTON mode
	// ----------------------------------------------------------------------
	@Override
	public void autonomousPeriodic() 
	{
		// =======================================
		// if not complete, this must run concurrently with all auton routines
		// =======================================
      	if(!_gearHandler.hasTiltAxisBeenZeroed())
    	{
      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
      		//			we must treat it as a Reentrant function
      		//			and automatically recall it until complete
    		_gearHandler.ZeroGearTiltAxisReentrant();
    	}
      	  	
      	// =======================================
		// Step 2: call the correct Auton routine to run
      	// =======================================
    	switch (_autonMode)
    	{
			case CROSS_BASE_LINE:
				if(_crossBaseLineAuton.getIsStillRunning())
				{
					_crossBaseLineAuton.ExecuteRentrant();
				}
				break;
				
			case DO_NOTHING:
				if(_doNothingAuton.getIsStillRunning())
				{
					_doNothingAuton.ExecuteRentrant();
				}
				break;
				
			case HANG_BOILER_SIDE_GEAR:
				if(_hangBoilerSideGearAuton.getIsStillRunning())
				{
					_hangBoilerSideGearAuton.ExecuteRentrant();
				}
				break;
				
			case HANG_CENTER_GEAR:
				if(_hangCenterGearAuton.getIsStillRunning())
				{
					_hangCenterGearAuton.ExecuteRentrant();
				}
				break;
				
			case HANG_KEY_SIDE_GEAR:
				if(_hangKeySideGear.getIsStillRunning())
				{
					_hangKeySideGear.ExecuteRentrant();
				}
				break;
				
			case TURN_AND_SHOOT:
				if(_turnAndShoot.getIsStillRunning())
				{
					_turnAndShoot.ExecuteRentrant();
				}
				break;
				
			case UNDEFINED:
			default:
				DriverStation.reportError("Error... NO AUTON Selected!", false);
				break;
    	}
		
    	// =====================================
    	// Step 3: Optionally Log Data
    	// =====================================
		WriteLogData();
	}

	// ----------------------------------------------------------------------
	// code executed 1x when entering TELOP Mode
	// ----------------------------------------------------------------------
	@Override
	public void teleopInit()
	{
    	// =====================================
    	// Step 1: Setup Robot Defaults
    	// =====================================
		
		// #### Chassis ####
    	//Stop motors
    	_chassis.FullStop();
  	
    	//Zero drive encoders
    	_chassis.ZeroDriveEncoders();
    	
    	//Set shifter to HIGH gear
    	_chassis.ShiftGear(GearShiftPosition.HIGH_GEAR);
    	
    	// disable acc/dec mode
    	_chassis.setIsAccDecModeEnabled(false);
    	_chassis.setDriveSpeedScalingFactor(1.0);
    	
    	// #### Climber ####
    	_climber.FullStop();
    	
    	// #### GearHandler ####
    	_gearHandler.FullStop();
    	if(!_gearHandler.hasTiltAxisBeenZeroed())	{ _gearHandler.ZeroGearTiltAxisInit(); }
    	
    	// #### Shooter ####
    	_shooter.FullStop();
    	_shooter.ActuatorMoveToDefaultPosition();
    	
    	// #### Ball Infeed ####
    	_ballInfeed.FullStop();
    	
    	// #### Cameras ####
    	// set to default camera
    	_switchableCameraServer.ChgToCamera(RobotMap.BALL_INFEED_CAMERA_NAME);
    	
    	// #### Telop Controller ####
    	_telopMode = TELEOP_MODE.STANDARD;	// default to std mode
    	
    	// #### Lidar starts doing ####
    	//if(_lidar != null)	{ _lidar.start(); }	//TODO: reslove timeout
    	
    	// =====================================
    	// Step 2: Configure Logging (if USB Memory Stick is present)
    	// =====================================    	
    	_dataLogger = GeneralUtilities.setupLogging("telop");
	}
	
	// ----------------------------------------------------------------------
	// Code executed every scan cycle (about every 20 mSec or 50x / sec) in TELOP Mode
	// ----------------------------------------------------------------------
	@Override
	public void teleopPeriodic() 
	{
    	// =====================================
    	// Step 0: Get the DASHBOARD Input values for the current scan cycle
    	// =====================================
    	_driversStation.ReadCurrentScanCycleValues();
    	
    	// =====================================
    	// Step 1: execute different steps based on current "telop mode"
    	// =====================================
    	switch (_telopMode)
    	{
    		case STANDARD:	    			
				//=====================
		    	// Gear Shift
				//=====================
		    	if(_driversStation.getIsDriver_GearShiftToggle_BtnJustPressed())
		    	{
		    		if (_chassis.getGearShiftPosition() == GearShiftPosition.HIGH_GEAR)
		    		{
		    			_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
		    		}
		    		else 
		    		{
		    			_chassis.ShiftGear(GearShiftPosition.HIGH_GEAR);
					}
		    	}
		    	
		    	//=====================
		    	// Acc/Dec Mode Toggle
				//=====================
		    	//if(_driversStation.getIsDriver_ToggleBlenderAndFeederMtrs_BtnJustPressed())  // TODO: fix this
		    	//{    		
		    	//	_chassis.setIsAccDecModeEnabled(!_chassis.getIsAccDecModeEnabled());
		    	//}
		    	
		    	//=====================
		    	// Chassis Throttle Cmd
				//=====================
		    	_chassis.ArcadeDrive(_driversStation.getDriver_ChassisThrottle_JoystickCmd(), 
		    							_driversStation.getDriver_ChassisTurn_JoystickCmd());
		    	
    			//============================================================================
    			// Fuel Infeed Cmd
    			//===========================================================================   			
    			if(_driversStation.getIsOperator_FuelInfeed_BtnPressed())
    			{
    				_ballInfeed.InfeedFuelAndExtendSolenoid();
    			}
    			else
    			{
    				_ballInfeed.FullStop();
    			}
    				
    			//=====================
    			// Handle Shooter Slider
    			//=====================
    			    			
    			if(_driversStation.getIsDriver_ShooterSliderUp_BtnJustPressed())
    			{
    				_shooter.ActuatorMoveUp();
    			}
    			if(_driversStation.getIsDriver_ShooterSliderDown_BtnJustPressed())
    			{
    				_shooter.ActuatorMoveDown();
    			}
    			
    			//=====================
    			// Run Shooter Motors
    			//=====================
    			
				// Stg 1 Cycle Up / Down
    			if(_driversStation.getIsDriver_ShooterStg1StepRPMUp_BtnJustPressed())
    			{
    				_shooter.Stg1MtrBumpRPMUp();
    			}
    			else if (_driversStation.getIsDriver_ShooterStg1StepRPMDown_BtnJustPressed())
				{
    				_shooter.Stg1MtrBumpRPMDown();
				}
    			
    			// Stg 2 Cycle Up / Down
    			if(_driversStation.getIsDriver_ShooterStg2StepRPMUp_BtnJustPressed())
    			{
    				_shooter.Stg2MtrBumpRPMUp();
    			}
    			else if (_driversStation.getIsDriver_ShooterStg2StepRPMDown_BtnJustPressed())
				{
    				_shooter.Stg2MtrBumpRPMDown();
				}
    			
    			// Stg 1 & 2 Full Stop
    			if(_driversStation.getIsDriver_FullShooterStop_BtnJustPressed())
    			{
    				_shooter.FullShooterStop();
    			}
    			   			      			
    			//=====================
    			// Blender and Feeder Motors
    			//=====================
    			if(_driversStation.getIsDriver_ToggleBlenderAndFeederMtrs_BtnJustPressed())
    			{
    				_shooter.ToggleSpinBlender();
    				_shooter.ToggleSpinFeeder();
    			}
    			else if (_driversStation.getIsDriver_BlenderCycleRPM_BtnJustPressed())
    			{
    				_shooter.BlenderMtrCycleVBus();
    			}   			
    			
		    	//=====================
		    	// Gear Tilt Cmd
		    	//	Note: All of the Gear Handler sequences are interruptable except for Zero!
				//=====================
		      	if(!_gearHandler.hasTiltAxisBeenZeroed())
		    	{
		      		// 1st priority is zeroing
		      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
		      		//			we must treat it as a Reentrant function
		      		//			and automatically recall it until complete
		    		_gearHandler.ZeroGearTiltAxisReentrant();
		    	}
		      	else if (_driversStation.getIsOperator_GearReZero_BtnJustPressed())
		      	{
		      		// 2nd priority is operator request to rezero
		      		_gearHandler.ZeroGearTiltAxisInit();	// zeroing will start on next scan
		      	}
		      	else if (Math.abs(_driversStation.getOperator_GearTiltFeed_JoystickCmd()) > 0.0)
		      	{
		      		// 3rd priority is joystick control
		      		_gearHandler.MoveTiltAxisVBus(_driversStation.getOperator_GearTiltFeed_JoystickCmd());
		      	}
		      	else if (_driversStation.getIsOperator_GearGoToHome_BtnJustPressed())
		      	{
		      		// 4th priority is Goto Home
		      		_gearHandler.MoveGearToHomePosition();
		      	}
		      	else if (_driversStation.getIsOperator_GearGoToScore_BtnJustPressed())
		      	{
		      		// 5th priority is Goto Score
		      		_gearHandler.MoveGearToScorePosition();
		      	}
		      	else if (_driversStation.getIsOperator_GearGoToFloor_BtnJustPressed()
		      				|| !_gearHandler.getIsLastTiltMoveToFloorCallComplete())
		      	{
		      		// 6th priority is Goto Floor
		      		// 	Note: MoveToFloor will take longer than 1 scan cycle to complete so
		      		//			we must treat it as a Reentrant function
		      		//			and automatically recall it until complete
		      		_gearHandler.MoveGearToFloorPositionReentrant();
		      	}  
		      	
		    	//=====================
		    	// Gear Infeed Cmd
				//=====================
		      	_gearHandler.SpinInfeedWheelsVBus(_driversStation.getOperator_GearInfeedOutFeed_JoystickCmd());
		      	
		    	//=======================================
		    	// Switchable Cameras
		    	//=====================================
				if(_driversStation.getIsOperator_CameraSwap_BtnJustPressed())
				{
					_switchableCameraServer.ChgToNextCamera();
				}
		    	
		    	//if(_driversStation.getIsOperator_CameraSwap_BtnJustPressed())
		    	//{
		        //	if(_switchableCameraServer.getCurrentCameraName() == RobotMap.GEAR_CAMERA_NAME)
		        //	{
		        //		_switchableCameraServer.ChgToCamera(RobotMap.SHOOTER_CAMERA_NAME);
		        //	}
		        //	
		        //	else if(_switchableCameraServer.getCurrentCameraName() == RobotMap.SHOOTER_CAMERA_NAME)
		        //	{
		        // 		_switchableCameraServer.ChgToCamera(RobotMap.BALL_INFEED_CAMERA_NAME);
		        //	}
		        //	
		        //	else if(_switchableCameraServer.getCurrentCameraName() == RobotMap.BALL_INFEED_CAMERA_NAME)
		        //	{
		        //		_switchableCameraServer.ChgToCamera(RobotMap.GEAR_CAMERA_NAME);
		        ///	}
		    	//}
		    	
		    	// =====================================
		    	// Check the climber
		    	// =====================================
		    	
		    	if (_driversStation.getIsOperator_StartClimb_ButtonJustPressed())
		    	{
		    		_telopMode = TELEOP_MODE.CLIMBING;
		    		_climber.RunClimberReentrant();
		    	}
		      	
				//=====================
		    	// Enter a Telop Sequence Mode
		      	//	Telop Sequences are small, pre-programmed, coordinated actions typically
		      	//	involving multiple subsystems and/or sensors
				//=====================
    			if(_driversStation.getIsOperator_GearStartSequence_BtnJustPressed())
    			{
    				// make sure Gear Tilt has completed zeroing before entering this mode!
    				if(_gearHandler.hasTiltAxisBeenZeroed())
    				{
	    				_telopMode = TELEOP_MODE.HANG_GEAR_SEQUENCE_MODE;
	    				_hangGearController.Initialize();
    				}
    				else
    				{
    					DriverStation.reportWarning("=!=!= Cannot chg to Hang Gear Seq, Tilt is NOT finished zeroing yet =!=!=", false);
    				}
    			}	
		      	
		      	break;	// end of _telopMode = STANDARD
      		
    		case HANG_GEAR_SEQUENCE_MODE:
    			
    			// in this teleop mode the driver & operator do not have control until
    			// the sequence completes or it times out
    			boolean isStillRunning = _hangGearController.ExecuteRentrant();
    			
    			// if not still running, switch back to std teleop mode
    			//	(ie: give control back to the driver & operator)
    			if(!isStillRunning)
    			{
    				_telopMode = TELEOP_MODE.STANDARD;
    			}
    			
    			break;	// end of _telopMode = HANG_GEAR_SEQUENCE_MODE
    			
    		case CLIMBING:
		    	if (_driversStation.getIsOperator_StartClimb_ButtonJustPressed())
		    	{	
		    		// cancel climbing
	    			_climber.FullStop();
	    			_telopMode = TELEOP_MODE.STANDARD;
		    	}
		    	else 
		    	{
		    		_climber.RunClimberReentrant();
		    	}
	    		
    			break;
    			
    	}	// end of switch statement
    	   	
    	// =====================================
    	// Step N: Finish up 
    	// =====================================
    	
    	// Refresh Dashboard
    	OutputAllToSmartDashboard();
    	
    	// =====================================
    	// Step N: Optionally Log Data
    	// =====================================
    	WriteLogData();
	}

	// ----------------------------------------------------------------------
	// Code executed every scan cycle (about every 20 mSec or 50x / sec) in TEST Mode
	// ----------------------------------------------------------------------
	@Override
	public void testPeriodic() 
	{
	}
	
	// ==================================================================================
	// General Helper Methods Follow
	// ==================================================================================
	
    // this method optionally calls the OutputToSmartDashboard on each subsystem
	//----------------------------------------------------------------------------------
	//  Utility / Helper Methods Follow
	//----------------------------------------------------------------------------------
	
    // utility method that calls the outputToSmartDashboard method on all subsystems
    private void OutputAllToSmartDashboard()
    {
    	if(_ballInfeed != null)		{ _ballInfeed.OutputToSmartDashboard(); }
    	
    	if(_chassis != null) 		{ _chassis.OutputToSmartDashboard(); }
    	
    	if(_climber != null)		{ _climber.OutputToSmartDashboard(); }
    	
    	if(_driversStation != null)	{ _driversStation.OutputToSmartDashboard(); }
    	
    	if(_gearHandler != null)	{ _gearHandler.OutputToSmartDashboard(); }
    		
    	if(_lidar != null)			{ _lidar.OutputToSmartDashboard(); }
    	
    	if(_navX != null)			{ _navX.OutputToSmartDashboard(); }
    	
    	if(_shooter != null)		{ _shooter.OutputToSmartDashboard(); }
    }
         
    // this method optionally calls the UpdateLogData on each subsystem and then logs the data
    private void WriteLogData()
    {    	
    	if(_dataLogger != null)
    	{    	
	    	// create a new, empty logging class
        	LogData logData = new LogData();
	    	
	    	// ask each subsystem that exists to add its data	    	
	    	if(_chassis != null) 		{ _chassis.UpdateLogData(logData); }
	    	
	    	if(_climber != null) 		{ _climber.UpdateLogData(logData); }
	    	
	    	if(_driversStation != null) { _driversStation.UpdateLogData(logData); }
	    	
	    	if(_gearHandler != null) 	{ _gearHandler.UpdateLogData(logData); }
	    	
	    	if(_ballInfeed != null) 	{ _ballInfeed.UpdateLogData(logData); }
	    	
	    	if(_lidar != null)			{ _lidar.UpdateLogData(logData); }
	    	
	    	if(_navX != null) 			{ _navX.UpdateLogData(logData); }
	    	
	    	if(_shooter != null)		{ _shooter.UpdateLogData(logData); }
    	
	    	// now write to the log file
	    	_dataLogger.WriteDataLine(logData);
    	}
    }
}