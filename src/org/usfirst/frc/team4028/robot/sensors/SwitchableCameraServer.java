package org.usfirst.frc.team4028.robot.sensors;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.opencv.core.Mat;
import org.usfirst.frc.team4028.robot.constants.RobotMap;
import org.usfirst.frc.team4028.robot.subsystems.DriversStation;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;

//This class implements all functionality for operator/driver cameras
//=====> For Changes see Nick Donahue (javadotmakeitwork)
public class SwitchableCameraServer
{	
	private String _cameraname;

	
	//============================================================================================
	// constructors follow
	//============================================================================================
	public SwitchableCameraServer(String cameraname)
	{
		_cameraname = cameraname;
		Thread cameraThread = new Thread(JavadotMakeItWork);
		cameraThread.start();
	}

	//============================================================================================
	// Methods follow
	//============================================================================================
	public void ChgToCamera(String cameraname) 
	{
		_cameraname = cameraname;
		System.out.println("Camera Swapped");
	}
	
	//============================================================================================
	// Property Accessors follow
	//============================================================================================
	public String getCurrentCameraName()
	{
		return _cameraname;
	}
	
	//============================================================================================
	// Task that is run on a separate thread
	//============================================================================================
	private Runnable JavadotMakeItWork = new Runnable()
	{
		@Override
		public void run() 
		{
	    	boolean isCam0Present = Files.exists(Paths.get("/dev/video0"));
	    	boolean isCam1Present = Files.exists(Paths.get("/dev/video1"));
	    	boolean isCam2Present = Files.exists(Paths.get("/dev/video2"));
	    	System.out.println("Cam0isConnected:" + isCam0Present);
	    	System.out.println("Cam1isConnected:" + isCam1Present);
	    	System.out.println("Cam2isConnected:" + isCam2Present);

	   
	    	UsbCamera cam0 = null;
	    	UsbCamera cam1 = null;
	    	UsbCamera cam2 = null;
	    	
	    	CvSink cvSink0 = null;
	    	CvSink cvSink1 = null;
	    	CvSink cvSink2 = null;
	    	
			// create instances of the all the cameras
            // create sinks for each camera
	    	if(isCam0Present)
	    	{
	    		cam0 = new UsbCamera(RobotMap.GEAR_CAMERA_NAME, 0); 				
	    		 cvSink0 = CameraServer.getInstance().getVideo(cam0); 
	    	}
	    	if(isCam1Present)
	    	{
	    		cam1 = new UsbCamera(RobotMap.SHOOTER_CAMERA_NAME,  1); 	
	    		cvSink1 = CameraServer.getInstance().getVideo(cam1);
	    	}
		    if(isCam2Present)
		    {
		    	cam2 = new UsbCamera(RobotMap.BALL_INFEED_CAMERA_NAME, 2); 
		    	cvSink2 = CameraServer.getInstance().getVideo(cam2);
		    }
 
            // create an output stream
            CvSource outputStream = CameraServer.getInstance().putVideo("Switcher", 640, 480);
            
            // create a 2d array to hold the captured image
            Mat image = new Mat();
            
            // create a MjpegServer on port 1181
            MjpegServer server = new MjpegServer("server", 1181);
            
            // set the image source for the mjepg server to be the output stream
            server.setSource(outputStream);
            
            boolean isImageAvailable = false;
            
            // start looping
            while(!Thread.interrupted()) 
            {
        		isImageAvailable = false;

        		if(_cameraname == "cam0")
        		{       
            		// disable the other cameras
            		//	NOTE: Key point is to disable all other cameras BEFORE you enable the one
            		//			you want to avoid USB bus overload!
        			if(isCam1Present)
        			{
            			cvSink1.setEnabled(false);
        			}
        			
        			if(isCam2Present)
        			{
            			cvSink2.setEnabled(false);
        			}
            		// enable this camera & configure it
        			if(isCam0Present)
        			{
		                cvSink0.setEnabled(true);
	                	cam0.setFPS(16);
	               		cam0.setResolution(640, 480);
	            		cvSink0.grabFrame(image);	     		// grab the current frame from this camera and put it into the 2D array
	            		isImageAvailable = true;
        			}

            	} 
            	else if(_cameraname == "cam1")
            	{
            		
        			if(isCam2Present)
        			{
            			cvSink2.setEnabled(false);
        			}
        			
        			if(isCam0Present)
        			{
            			cvSink0.setEnabled(false);
        			}
            		// enable this camera & configure it
        			if(isCam1Present)
        			{
		                cvSink1.setEnabled(true);
	                	cam1.setFPS(16);
	               		cam1.setResolution(640, 480);
	            		cvSink1.grabFrame(image);	     		// grab the current frame from this camera and put it into the 2D array
	            		isImageAvailable = true;
        			}
            		
            	}
            	else if(_cameraname =="cam2")
            	{
        			if(isCam0Present)
        			{
            			cvSink0.setEnabled(false);
        			}
        			
        			if(isCam1Present)
        			{
            			cvSink1.setEnabled(false);
        			}
            		// enable this camera & configure it
        			if(isCam2Present)
        			{
		                cvSink2.setEnabled(true);
	                	cam2.setFPS(16);
	               		cam2.setResolution(640, 480);
	            		cvSink2.grabFrame(image);	     		// grab the current frame from this camera and put it into the 2D array
	            		isImageAvailable = true;

        			}
            	}

	           	// push the captured frame to the output stream
           		if(isImageAvailable)
           		{
           			outputStream.putFrame(image);
           		}


            }
	            	
		}
	};
}