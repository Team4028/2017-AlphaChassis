package org.usfirst.frc.team4028.robot.sensors;

/*
 * I was attempting to...
 * 
 * 1. Only send necessary distance values and make # the starting value and $ the ending value ("5#0034#0035#" becomes "5$#34$#35$
 * 1b. find a way to get the position of #(starting value) and $(ending value) in the string
 * 1c. then use those positions to find and extract the values between them
 * 1d. then throw out all of the extracted characters except for ["1", "2", "3", "4", "5", "6", "7", "8", "9"]
 * 
 * 2. find a way to not record values when we do not have any... by either simply not recording or replace with the previous value
 */
import edu.wpi.first.wpilibj.SerialPort;
import java.util.TimerTask;
import java.lang.String;

public class LIDAR {
	private java.util.Timer _updaterTimer;
	private LIDARUpdaterTask _task;
	private boolean _isRunning = false;
	private  int _distance;
	SerialPort _serialport;
	
	
	public LIDAR(SerialPort.Port PortConstant)
	{
		_task = new LIDARUpdaterTask();
		_serialport = new SerialPort(9600, PortConstant);
		_updaterTimer = new java.util.Timer();	
	}
	
	public void start()
	{
		start(20);
	}
	
	public void start(int period)
	{
		_isRunning = true;
		_updaterTimer.scheduleAtFixedRate(_task, 0, period);
	}
	
	public void stop()
	{
		_isRunning = false;
		_updaterTimer.cancel();
	}
	
	public void update()		// UPDATE DISTANCE VAR
	{
		try
		{
			String _RawDistanceText = _serialport.readString();
			//int #pos = get(_RawDistanceText.charAt
	
	
			//System.out.println(_serialport.readString());
			//String _RawDistanceText = "#0035";
			
			//if(_RawDistanceText.startsWith('#'))
			if(_RawDistanceText.charAt(1) == '#'){
				_RawDistanceText = _RawDistanceText.substring(2);
	//			   replace charAt(0)
	//			_RawDistanceText.charAt(0);
	//			 public static String removeCharAt(String s, int pos) {
	//			      return s.substring(0, pos) + s.substring(pos + 1);
	//			   }
			}
			_distance = Integer.parseInt(_RawDistanceText);
		}
		catch (NumberFormatException e)
		{
			e.printStackTrace();
		}
		catch (StringIndexOutOfBoundsException e)
		{
			e.printStackTrace();
		}
	}
	
	public int get_Distance ()
	{
		return _distance;
	}

	private class LIDARUpdaterTask extends TimerTask
	{
		public void run()
		{
			while (_isRunning)
			{
				update();
				
				try
				{
					Thread.sleep(10); //10 MS
				}
				catch (InterruptedException e)
				{
					e.printStackTrace();
				}
			}
		}
	}
}

