package org.usfirst.frc.team4028.robot.util;

import org.usfirst.frc.team4028.robot.util.GeneratedTrajectory;
import org.usfirst.frc.team4028.robot.util.Trajectory;

public class TrajectoryFollower {
	
	private double _kp;
	private double _ki; // Likely will not be used, currently not implemented
	private double _kd;
	private double _kv;
	private double _ka;
	private double _currentHeading;
	private double _lastError;
	private int _currentSegment;
	private Trajectory _profile;	
	public String _followerName;
	
	public TrajectoryFollower(String name) {
		this._followerName = name;
	}
	
	public void configure(double kp, double ki, double kd, double kv, double ka) {
		_kp = kp;
		_ki = ki;
		_kd = kd;
		_kv = kv;
		_ka = ka;
	}
	
	public void reset() {
		_lastError = 0.0;
		_currentSegment = 0;
	}
	
	public double calculate(double distanceSoFar, double[][] motionProfile, int currentSegment) {
		if (currentSegment < GeneratedTrajectory.kNumPoints) {
	      double error = motionProfile[currentSegment][0] - distanceSoFar;
	      double output = _kp * error + _kd * ((error - _lastError)
	              / motionProfile[currentSegment][4] - motionProfile[currentSegment][1]) + (_kv * motionProfile[currentSegment][1]
	              + _ka * motionProfile[currentSegment][2]);
	
	      _lastError = error;
	      _currentHeading = motionProfile[currentSegment][3];
	      _currentSegment = currentSegment;
	      return output;
	    } else {
	      return 0;
	    }
	}
	
	public boolean isTrajectoryFinished() {
		return _currentSegment >= GeneratedTrajectory.kNumPoints;
	}
	
	public void setTrajectory(Trajectory profile) {
		_profile = profile;
	}
	
	public double getHeading() {
		return _currentHeading;
	}
	
	public int getCurrentSegment() {
		return _currentSegment;
	}
	
	public int getNumSegments() {
		return _profile.getNumSegments();
	}
}
