package org.usfirst.frc.team4028.robot.sensors;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.Date;
import java.util.TimerTask;
import java.util.Vector;

import org.usfirst.frc.team4028.robot.vision.Dimension;
import org.usfirst.frc.team4028.robot.vision.RoboRealmAPI;
import org.usfirst.frc.team4028.robot.vision.RawImageData;

//import uk.co.geolib.geopolygons.*;
//import uk.co.geolib.geolib.*;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * This class is a wrapper around the interface to RoboRealm over TCP Sockets hosted on a 
 * on-robot Kangaroo
 */
public class RoboRealmClient 
{
	// Define local working variables
 	private RoboRealmAPI _rrAPI;
 	
	private java.util.Timer _updaterTimer; 
 	private RoboRealmUpdater _task; 

 	private double _fovCenterToTargetXAngleRawDegrees;
 	private boolean _isConnected;
 	
 	private static final int NORTHWEST_X_IDX = 0;
 	private static final int NORTHEAST_X_IDX = 1;
 	private static final int HIGHESTMIDDLE_Y_IDX = 2;
 	private static final int BLOB_COUNT_IDX = 3;
 	
 	private static final int EXPECTED_ARRAY_SIZE = 4;
 	
 	private static final double CAMERA_FOV_HORIZONTAL_DEGREES = 58.5;
 	
 	// Constructor
    public RoboRealmClient(String kangarooIPv4Addr, int portNo) 
    {        	
    	// create an instance of the RoboRealm API client
    	_rrAPI = new RoboRealmAPI();
    	
    	// try to connect
        if (!_rrAPI.connect(kangarooIPv4Addr, portNo))
        {
        	DriverStation.reportError("Could not connect to RobeRealm!", false);
        	System.out.println("====> Could not connect to RobeRealm!");
        	
        	_isConnected = false;
        }
        else
        {
        	DriverStation.reportWarning("Connected to RobeRealm!", false);
        	System.out.println("====> Connected to RobeRealm!");
        	
        	_isConnected = true;
        }
        
		// create instance of the task the timer interrupt will execute
		_task = new RoboRealmUpdater();
		
		// create a timer to fire events
		_updaterTimer = new java.util.Timer();
    }
    
    public void TestConnect()
    {
    	Dimension fovDimensions = _rrAPI.getDimension();
    }
    
 	public double getAngle() 
 	{ 
 		return _fovCenterToTargetXAngleRawDegrees; 
 	} 
    
 	// this method switches the currently running pipeline program
 	public void SwitchProgram(String pipeLineProgramFullPathName)
 	{
 		Boolean isPauseOk = _rrAPI.pause();
 		
 		System.out.println("====> Prepare to Switch Program");
 		
 		if(isPauseOk)
        {
        	//DriverStation.reportError("RoboRealm Program Paused succesfully!", false);
        	System.out.println("====> RoboRealm Program Paused succesfully!");
        }
        else
        {
        	//DriverStation.reportError("Error..Could not pause RoboRealm program!", false);
        	System.out.println("====>Error..Could not pause RoboRealm program!");
        }
 		
 		System.out.println("====> Chg Program To: " + pipeLineProgramFullPathName);
 		
 		Boolean isSwitchOk = _rrAPI.loadProgram(pipeLineProgramFullPathName);
 		
 		if(isSwitchOk)
        {
        	//DriverStation.reportError("RoboRealm Program Switched succesfully to: " + pipeLineProgramFullPathName, false);
        	System.out.println("====> RoboRealm Program Switched succesfully to: " + pipeLineProgramFullPathName);
        }
        else
        {
        	//DriverStation.reportError("Error..Could not switch RoboRealm program!", false);
        	System.out.println("====> Error..Could not switch RoboRealm program!");
        }	
 		
 		Boolean isResumeOk = _rrAPI.resume();
 		
 		if(isResumeOk)
        {
        	//DriverStation.reportError("RoboRealm Program Paused succesfully!", false);
        	System.out.println("====> RoboRealm Program Resumed succesfully!");
        }
        else
        {
        	//DriverStation.reportError("Error..Could not pause RoboRealm program!", false);
        	System.out.println("====> Error..Could not resume RoboRealm program!");
        }
 	}
 	
	// poll RoboRealm to read current values
 	public void update() 
 	{ 
 		long startOfCallTimestamp = new Date().getTime();
 		
 		// get the Field Of View Dimensions
 		Dimension fovDimensions = _rrAPI.getDimension();
 		
 	    // get multiple variables
 		// This must match what is in the config of the "Point Location" pipeline step in RoboRealm
 	    Vector v = _rrAPI.getVariables("NORTHWEST_X,NORTHEAST_X,HIGHEST_MIDDLE_Y,BLOB_COUNT");
 	    
 	    long callElapsedTimeMSec = new Date().getTime() - startOfCallTimestamp;
 	    
 	    if (v==null)
 	    {
 	    	DriverStation.reportError("Error in GetVariables, did not return any results", false);
 	    }
 	        
 	    RawImageData newTargetRawData = null;
 	    if(v.size() == EXPECTED_ARRAY_SIZE)
 	    {
 	    	newTargetRawData = new RawImageData();
 	    	
 	    	// parse the results and build the image data
 	    	newTargetRawData.Timestamp = new Date().getTime();
 	    	newTargetRawData.NorthWestX = Integer.parseInt((String)v.elementAt(NORTHWEST_X_IDX));
 	    	newTargetRawData.NorthEastX = Integer.parseInt((String)v.elementAt(NORTHEAST_X_IDX));
 	    	newTargetRawData.HighestMiddleY = Integer.parseInt((String)v.elementAt(HIGHESTMIDDLE_Y_IDX));
 	    	newTargetRawData.BlobCount = Integer.parseInt((String)v.elementAt(BLOB_COUNT_IDX));
 	    	newTargetRawData.FOVDimensions = fovDimensions;
 	    	
 	    	newTargetRawData.ResponseTimeMSec = callElapsedTimeMSec;
 	    	
 	    	/*
 	    	System.out.println("NWx: " + newTargetRawData.NorthWestX 
 	    						+ " NEx: " + newTargetRawData.NorthEastX 
 	    						+ " HMY: " + newTargetRawData.HighestMiddleY 
 	    						+ " Blob: " + newTargetRawData.BlobCount
 	    						+ " mSec: " + newTargetRawData.ResponseTimeMSec);
 	    	*/
 	    	
 	    	// create a point at the center of the field of view
 	    	int fovXCenterPoint = fovDimensions.width / 2;
 	    	
 	    	// calc target center point
 	    	int targetXCenterPoint = (newTargetRawData.NorthEastX - newTargetRawData.NorthWestX) / 2;
 	    		
 	    	double fovCenterToTargetXAngleRawDegrees = (fovXCenterPoint / fovDimensions.width) * CAMERA_FOV_HORIZONTAL_DEGREES;
 	    		
 	    		_fovCenterToTargetXAngleRawDegrees = Math.round(fovCenterToTargetXAngleRawDegrees * 100.0)/ 100.0;
 	    		System.out.println("Angle= " + _fovCenterToTargetXAngleRawDegrees + " mSec=" + newTargetRawData.ResponseTimeMSec);
 	    }
 	    else
 	    {
 	    	newTargetRawData = null;
 	    }
 	} 
 	
    private class RoboRealmUpdater extends TimerTask 
    { 
 		public void run() 
 		{ 
 			while(true) 
 			{ 
 				update(); 
 				
 				try 
 				{ 
 					// sleep for 10 mSec
 					Thread.sleep(10); 
				} 
 				catch (InterruptedException e) 
 				{ 
 					e.printStackTrace(); 
				} 
 			} 
 		} 
 	}
}
