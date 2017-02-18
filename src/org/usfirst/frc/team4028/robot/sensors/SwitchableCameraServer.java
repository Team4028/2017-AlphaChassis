package org.usfirst.frc.team4028.robot.sensors;

import org.opencv.core.Mat;
import org.usfirst.frc.team4028.robot.constants.RobotMap;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj.CameraServer;

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
			// create instances of the all the cameras
			UsbCamera _cam0 = new UsbCamera(RobotMap.GEAR_CAMERA_NAME, 0); 			//CameraServer.getInstance().startAutomaticCapture(0);		
            UsbCamera _cam1 = new UsbCamera(RobotMap.SHOOTER_CAMERA_NAME,  1); 		//CameraServer.getInstance().startAutomaticCapture(1);           
            UsbCamera _cam2 = new UsbCamera(RobotMap.BALL_INFEED_CAMERA_NAME, 2); 	//CameraServer.getInstance().startAutomaticCapture(1);
           
            // create sinks for each camera
            CvSink cvSink1 = CameraServer.getInstance().getVideo(_cam0);
            CvSink cvSink2 = CameraServer.getInstance().getVideo(_cam1);
            CvSink cvSink3 = CameraServer.getInstance().getVideo(_cam2);
            
            // create an output stream
            CvSource outputStream = CameraServer.getInstance().putVideo("Switcher", 640, 480);
            
            // create a 2d array to hold the captured image
            Mat image = new Mat();
            
            // create a MjpegServer on port 1181
            MjpegServer server = new MjpegServer("server", 1181);
            
            // set the image source for the mjepg server to be the output stream
            server.setSource(outputStream);
            
            // start looping
            while(!Thread.interrupted()) 
            {
            	if(_cameraname == "cam0")
            	{       
            		// disable the other cameras
            		//	NOTE: Key point is to disable all other cameras BEFORE you enable the one
            		//			you want to avoid USB bus overload!
            		cvSink2.setEnabled(false);
            		cvSink3.setEnabled(false);
            		
            		// enable this camera & configure it
            		cvSink1.setEnabled(true);
            		_cam0.setFPS(16);
            		_cam0.setResolution(640, 480);
            		
            		// grab the current frame from this camera and put it into the 2D array
            		cvSink1.grabFrame(image);
            	} 
            	else if(_cameraname == "cam1")
            	{
            		cvSink1.setEnabled(false);
            		cvSink3.setEnabled(false);
            		
            		cvSink2.setEnabled(true);
            		_cam1.setFPS(16);
            		_cam1.setResolution(640, 480);
            		
            		cvSink2.grabFrame(image);	
            	}
            	else if(_cameraname =="cam2")
            	{
            		cvSink1.setEnabled(false);
            		cvSink2.setEnabled(false);
            		
            		cvSink3.setEnabled(true);
            		_cam2.setFPS(16);
            		_cam2.setResolution(640, 480);
            		
            		cvSink3.grabFrame(image);
            	}
                
            	// push the captured frame to the output stream
                outputStream.putFrame(image);
            }
		}
	};
}