package org.usfirst.frc.team4028.robot;

import org.usfirst.frc.team4028.robot.constants.GeneralEnums.TELEOP_MODE;
import org.usfirst.frc.team4028.robot.constants.RobotMap;
import org.usfirst.frc.team4028.robot.sensors.Lidar;
import org.usfirst.frc.team4028.robot.sensors.NavXGyro;
import org.usfirst.frc.team4028.robot.sensors.SwitchableCameraServer;
import org.usfirst.frc.team4028.robot.sequences.HangGearInTeleopSequence;
import org.usfirst.frc.team4028.robot.subsystems.Chassis;
import org.usfirst.frc.team4028.robot.subsystems.Chassis.GearShiftPosition;
import org.usfirst.frc.team4028.robot.subsystems.Climber;
import org.usfirst.frc.team4028.robot.subsystems.DashboardInputs;
import org.usfirst.frc.team4028.robot.subsystems.DriversStation;
import org.usfirst.frc.team4028.robot.subsystems.GearHandler;
import org.usfirst.frc.team4028.robot.subsystems.BallInfeed;
import org.usfirst.frc.team4028.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;

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
	
	// ===========================================================
	//   Define class level instance variables for Robot Telep Sequences 
	// ===========================================================
	private HangGearInTeleopSequence _hangGearInTeleopSeq;
	
	// ===========================================================
	//   Define class level instance variables for Robot Auton Routines 
	// ===========================================================
	
	// ----------------------------------------------------------------------
	// Code executed 1x at robot startup
	// ----------------------------------------------------------------------
	@Override
	public void robotInit() 
	{
        //===================
    	// write jar (build) date & time to the dashboard
        //===================
    	Utilities.WriteBuildInfoToDashboard(ROBOT_NAME);
    	
        //===================
    	// create instances (and configure) all of all robot subsystems & sensors
        //===================
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
		
		_ballInfeed = new BallInfeed(RobotMap.INFEED_MTR_CAN_BUS_ADDR, RobotMap.PCM_CAN_BUS_ADDR, RobotMap.BALL_INFEED_TILT_EXTEND_PCM_PORT);
		
		_shooter = new Shooter(RobotMap.SHOOTER_STG1_CAN_BUS_ADDR, 
								RobotMap.SHOOTER_STG2_CAN_BUS_ADDR,
								RobotMap.BLENDER_CAN_BUS_ADDR,
								RobotMap.FEEDER_CAN_BUS_ADDR,
								RobotMap.SHOOTER_SLIDER_PWM_PORT);
		
		// sensors follow
		_lidar = new Lidar();
		//_navX = new NavXGyro(RobotMap.NAVX_PORT);
		_switchableCameraServer = new SwitchableCameraServer(RobotMap.GEAR_CAMERA_NAME);
		
		// telop sequences follow
		_hangGearInTeleopSeq = new HangGearInTeleopSequence(_gearHandler, _chassis);
		
		// auton routines follow
		
	}
	
	// ----------------------------------------------------------------------
	// called each time the robot enters disabled mode from either telop or auton mode
	// ----------------------------------------------------------------------
	@Override
	public void disabledInit() 
	{
		// if logging was enabled make sure we close the file
    	if(_dataLogger != null)
    	{
	    	_dataLogger.close();
	    	_dataLogger = null;
    	}
	}
		
	// ----------------------------------------------------------------------
	// code executed 1x when entering AUTON Mode
	// ----------------------------------------------------------------------
	@Override
	public void autonomousInit() 
	{
		// TODO: add logic to read from Dashboard Choosers to select the Auton routine to run
		
    	// #### GearHandler ####
    	_gearHandler.FullStop();
    	if(!_gearHandler.hasTiltAxisBeenZeroed())
    	{
    		_gearHandler.ZeroGearTiltAxisInit();
    	}
		
    	// =====================================
    	// Step N: Optionally Configure Logging
    	// =====================================
    	_dataLogger = Utilities.setupLogging("auton");
	}

	// ----------------------------------------------------------------------
	// Code executed every scan cycle (about every 20 mSec or 50x / sec) in AUTON mode
	// ----------------------------------------------------------------------
	@Override
	public void autonomousPeriodic() 
	{
		// this can run concurrently with all auton routines
      	if(!_gearHandler.hasTiltAxisBeenZeroed())
    	{
      		// 	Note: Zeroing will take longer than 1 scan cycle to complete so
      		//			we must treat it as a Reentrant function
      		//			and automatically recall it until complete
    		_gearHandler.ZeroGearTiltAxisReentrant();
    	}
      	
      	
		// TODO: add logic to call the correct Auton routine to run
		
		
		
    	// =====================================
    	// Step N: Optionally Log Data
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
    	
    	//Set shifter to LOW gear
    	_chassis.ShiftGear(GearShiftPosition.LOW_GEAR);
    	
    	// disable acc/dec mode
    	_chassis.setIsAccDecModeEnabled(false);
    	//_chassis.setDriveSpeedScalingFactor(1.0);
    	
    	// #### Climber ####
    	_climber.FullStop();
    	
    	// #### GearHandler ####
    	_gearHandler.FullStop();
    	if(!_gearHandler.hasTiltAxisBeenZeroed())
    	{
    		_gearHandler.ZeroGearTiltAxisInit();
    	}
    	
    	// #### Shooter ####
    	_shooter.FullStop();
    	_shooter.ActuatorInitialConfig();
    	
    	// #### Ball Infeed ####
    	_ballInfeed.FullStop();
    	
    	// #### Cameras ####
    	//_switchableCameraServer.ChgToCamera(RobotMap.BALL_INFEED_CAMERA_NAME);
    	
    	// #### Telop Sequences ####
    	_telopMode = TELEOP_MODE.STANDARD;	// default to std mode
    	
    	// =====================================
    	// Step N: Configure Logging (if USB Memory Stick is present)
    	// =====================================    	
    	_dataLogger = Utilities.setupLogging("telop");
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
		    	if(_driversStation.getIsDriver_AccDecModeToggle_BtnJustPressed())
		    	{    		
		    		_chassis.setIsAccDecModeEnabled(!_chassis.getIsAccDecModeEnabled());
		    	}
		    	
		    	//=====================
		    	// Chassis Throttle Cmd
				//=====================
		    	_chassis.Drive(_driversStation.getDriver_ChassisThrottle_JoystickCmd(), 
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
    			
    			
    			//===========================================================================
    			//Switchable Cameras
    			//=======================================================================
    			
    			if(_driversStation.getIsOperator_CameraSwap_BtnJustPressed())
    			{
    				
    				if(_switchableCameraServer.getCurrentCameraName() == RobotMap.GEAR_CAMERA_NAME)
    				{
    					_switchableCameraServer.ChgToCamera(RobotMap.SHOOTER_CAMERA_NAME);
    				}
    				else if(_switchableCameraServer.getCurrentCameraName() == RobotMap.SHOOTER_CAMERA_NAME)
    				{
    					_switchableCameraServer.ChgToCamera(RobotMap.BALL_INFEED_CAMERA_NAME);
    				}
    				else if(_switchableCameraServer.getCurrentCameraName() == RobotMap.BALL_INFEED_CAMERA_NAME)
    				{
    					_switchableCameraServer.ChgToCamera(RobotMap.GEAR_CAMERA_NAME);
    				}
    				
    			}
		    	  
    			//=====================
    			// Run Shooter Motors (TEST)
    			//=====================
    			
    			if(_driversStation.getIsDriver_ShooterStg1Up_BtnJustPressed())
    			{
    				_shooter.Stg1RPMUp();
    			}
    			if(_driversStation.getIsDriver_ShooterStg1Down_BtnJustPressed())
    			{
    				_shooter.Stg1RPMDown();
    			}
    			if(_driversStation.getIsDriver_ShooterStg2Up_BtnJustPressed())
    			{
    				_shooter.Stg2RPMUp();
    			}
    			if(_driversStation.getIsDriver_ShooterStg2Down_BtnJustPressed()) 
    			{
    				_shooter.Stg2RPMDown();		
    			}
    			if(_driversStation.getIsDriver_FullShooterStop_BtnJustPressed())
    			{
    				_shooter.FullStop();
    			}
    			
    			//=====================
    			// Blender and Feeder Motors
    			//=====================
    			if(_driversStation.getIsDriver_AccDecModeToggle_BtnJustPressed())
    			{
    				_shooter.SpinBlender();
    				_shooter.SpinFeeder();
    			}
    			
    			//=====================
    			// Handle Actuator
    			//=====================
    			    			
    			if(_driversStation.getIsDriver_ActuatorUp_BtnJustPressed())
    			{
    				_shooter.ActuatorUp();
    			}
    			if(_driversStation.getIsDriver_ActuatorDown_BtnJustPressed())
    			{
    				_shooter.ActuatorDown();
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
	    				_hangGearInTeleopSeq.Initialize();
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
    			boolean isStillRunning = _hangGearInTeleopSeq.ExecuteSequenceRentrant();
    			
    			// if not still running, switch back to std teleop mode
    			//	(ie: give control back to the driver & operator)
    			if(!isStillRunning)
    			{
    				_telopMode = TELEOP_MODE.STANDARD;
    			}
    			
    			break;	// end of _telopMode = HANG_GEAR_SEQUENCE_MODE
    			
    	}	// end of switch statement
    	
    	//=======================================
    	//Cameras
    	//=====================================
    	if(_driversStation.getIsOperator_CameraSwap_BtnJustPressed())
    	{
        	if(_switchableCameraServer.getCurrentCameraName() == RobotMap.GEAR_CAMERA_NAME)
        	{
        		_switchableCameraServer.ChgToCamera(RobotMap.SHOOTER_CAMERA_NAME);
        	}
        	
        	else if(_switchableCameraServer.getCurrentCameraName() == RobotMap.SHOOTER_CAMERA_NAME)
        	{
        		_switchableCameraServer.ChgToCamera(RobotMap.BALL_INFEED_CAMERA_NAME);
        	}
        	
        	else if(_switchableCameraServer.getCurrentCameraName() == RobotMap.BALL_INFEED_CAMERA_NAME)
        	{
        		_switchableCameraServer.ChgToCamera(RobotMap.GEAR_CAMERA_NAME);
        	}
    	}
    	
    	// =====================================
    	// Step 5: Check the climber
    	// =====================================
    	
    	if (_driversStation.getIsOperator_StartClimb_ButtonJustPressed())
    	{
    		if (!_climber.getIsClimbing())
    		{
    			_climber.StartClimber();
    		}
    		else 
    		{
    			_climber.FullStop();
    		}	
    	}
    	else 
    	{
    		if (_climber.getIsClimbing())
    		{
    			_climber.StartClimber();
    		}
    	}
      	
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
    	if(_chassis != null)
    	{
    		_chassis.OutputToSmartDashboard();
    	}
    	
    	if(_climber != null)
    	{
    		_climber.OutputToSmartDashboard();
    	}
    	
    	if(_driversStation != null)
    	{
    		_driversStation.OutputToSmartDashboard();
    	}
    	
    	if(_gearHandler != null)
    	{
    		_gearHandler.OutputToSmartDashboard();
    	}
    	
    	if(_ballInfeed != null)
    	{
    		//_ballInfeed.OutputToSmartDashboard();			//TODO fix this, smart dashboard throwing unhandled exceptions
    	}
    	
    	if(_lidar != null)
    	{
    		_lidar.OutputToSmartDashboard();
    	}
    	
    	if(_navX != null)
    	{
    		_navX.OutputToSmartDashboard();
    	}
    	
    	if(_shooter != null)
    	{
    		_shooter.OutputToSmartDashboard();
    	}
    }
         
    // this method optionally calls the UpdateLogData on each subsystem and then logs the data
    private void WriteLogData()
    {    	
    	if(_dataLogger != null)
    	{    	
	    	// create a new, empty logging class
        	LogData logData = new LogData();
	    	
	    	// ask each subsystem that exists to add its data	    	
	    	if(_chassis != null)
	    	{
	    		_chassis.UpdateLogData(logData);
	    	}
	    	
	    	if(_climber != null)
	    	{
	    		// TODO: Temporarily commented out
	    		_climber.UpdateLogData(logData);
	    	}
	    	
	    	if(_driversStation != null)
	    	{
	    		_driversStation.UpdateLogData(logData);
	    	}
	    	
	    	if(_gearHandler != null)
	    	{
	    		_gearHandler.UpdateLogData(logData);
	    	}
	    	
	    	if(_ballInfeed != null)
	    	{
	    		_ballInfeed.UpdateLogData(logData);
	    	}
	    	
	    	if(_lidar != null)
	    	{
	    		_lidar.UpdateLogData(logData);
	    	}
	    	
	    	if(_navX != null)
	    	{
	    		_navX.UpdateLogData(logData);
	    	}
	    	
	    	if(_shooter != null)
	    	{
	    		_shooter.UpdateLogData(logData);
	    	}
    	
	    	// now write to the log file
	    	_dataLogger.WriteDataLine(logData);
    	}
    }

}