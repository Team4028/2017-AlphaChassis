package org.usfirst.frc.team4028.robot.constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

// This class contains id values for the physical elements of the robot so we can use names in the code
// instead of hardcoded constants
public class RobotMap 
{
	// ======================================
	// Driver's station
	// ======================================
	public static final int DRIVER_GAMEPAD_USB_PORT = 0;
	public static final int OPERATOR_GAMEPAD_USB_PORT = 1;
	
	// ======================================
	// Constants for CAN Bus Addresses
	// ======================================
	
	// define constant for PCM (Pneumatic Control Module)
	public static final int PCM_CAN_BUS_ADDR = 0;				
	
	// define constant for PDP (Power Distribution Panel) CAN Bus Address
	public static final int PDB_CAN_BUS_ADDR = 0; 
	
	// define constants for Talon SRX CAN Bus Addresses
	public static final int LEFT_MASTER_CAN_BUS_ADDR = 11;
	public static final int LEFT_SLAVE1_CAN_BUS_ADDR = 12;

	public static final int RIGHT_MASTER_CAN_BUS_ADDR = 9;
	public static final int RIGHT_SLAVE1_CAN_BUS_ADDR = 10;
	
	public static final int CLIMBER_CAN_BUS_ADDR = 4;
	
	public static final int GEAR_TILT_CAN_BUS_ADDR = 7;
	public static final int GEAR_INFEED_CAN_BUS_ADDR = 8;
	
	public static final int SHOOTER_STG1_CAN_BUS_ADDR = 1;
	public static final int SHOOTER_STG2_CAN_BUS_ADDR = 2;
	public static final int BLENDER_CAN_BUS_ADDR = 6;
	public static final int FEEDER_CAN_BUS_ADDR = 5;
	
	public static final int FUEL_INFEED_MTR_CAN_BUS_ADDR = 3;
	
	// ======================================
	// define constants for PWM Ports on RobioRio
	// ======================================
	public static final int SHOOTER_SLIDER_PWM_PORT = 9;
	
	// ======================================
	// define constants for DIO Ports on RoboRio
	// ======================================
	
	// ======================================
	// define constants for Sensor Ports on RoboRio
	// ======================================
	public static final SPI.Port NAVX_PORT = Port.kMXP;
	
	// ======================================
	// define constants for usb cameras
	// ======================================
	public static final String GEAR_CAMERA_NAME = "cam0";
	public static final String BALL_INFEED_CAMERA_NAME = "cam1";
	public static final String SHOOTER_CAMERA_NAME = "cam2";
	public static final String CLIMBER_CAMERA_NAME = "cam3";
	
	// ======================================
	// Define constants for solenoid ports on Pneumatic Control Module (PCM)
	// ======================================
	public static final int SHIFTER_SOLENOID_RETRACT_PCM_PORT = 7;
	public static final int SHIFTER_SOLENOID_EXTEND_PCM_PORT = 6;
	
	// TODO: update these
	public static final int BALL_INFEED_TILT_EXTEND_PCM_PORT = 5;
	
	// ======================================
	// Define constants for solenoid positions 
	// ======================================
	public static final Value SHIFTER_SOLENOID_LOW_GEAR_POSITION = DoubleSolenoid.Value.kForward;
	public static final Value SHIFTER_SOLENOID_HIGH_GEAR_POSITION = DoubleSolenoid.Value.kReverse;
	
	// TODO: validate these
	//2/18/17 Don't believe these are necessary Nick Donahue
	//public static final Value BALL_INFEED_TILT_RETRACT_POSITION = DoubleSolenoid.Value.kForward;
	//public static final Value BALL_INFEED_TILT_EXTEND_POSITION = DoubleSolenoid.Value.kReverse;
	
	// ======================================
	// define constants for logging
	// ======================================
	// this is where the USB stick is mounted on the RoboRIO filesystem.  You can confirm by logging into the RoboRIO using WinSCP
	public static final String PRIMARY_LOG_FILE_PATH = "/media/sda1/logging";
	public static final String ALTERNATE_LOG_FILE_PATH = "/media/sdb1/logging";
	
	// ======================================
	// define constants for the Kangaroo & RoboRealm
	// ======================================
	public static final String KANGAROO_IPV4_ADDR = "10.40.28.11";
	public static final int RR_API_PORT = 6060;
}
