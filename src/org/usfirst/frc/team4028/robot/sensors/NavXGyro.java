package org.usfirst.frc.team4028.robot.sensors;

import org.usfirst.frc.team4028.robot.LogData;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

// this class encapsulates interactions with the NavX Sensor
// you must setup path to libraries using these instructions:
//	http://www.pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/
// http://www.pdocs.kauailabs.com/navx-mxp/examples/rotate-to-angle-2/
//=====> For Changes see TBD
public class NavXGyro 
{
	AHRS _navXSensor;
	
	public NavXGyro(SPI.Port port)
	{
        try 
        {
			/***********************************************************************
			 * navX-MXP:
			 * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.            
			 * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported.
			 ************************************************************************/
        	_navXSensor = new AHRS(port); 
        } 
        catch (RuntimeException ex ) 
        {
            DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
        }
	}
	
	
	//============================================================================================
	// Methods follow
	//============================================================================================
	
	// update the Dashboard with any NavX specific data values
	public void OutputToSmartDashboard()
	{
		
	}
	
	public void UpdateLogData(LogData logData)
	{
		
	}
}
