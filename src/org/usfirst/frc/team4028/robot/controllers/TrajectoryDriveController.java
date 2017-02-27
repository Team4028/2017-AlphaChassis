package org.usfirst.frc.team4028.robot.controllers;

import org.usfirst.frc.team4028.robot.util.BeefyMath;
import org.usfirst.frc.team4028.robot.util.Trajectory;
import org.usfirst.frc.team4028.robot.util.TrajectoryFollower;
import org.usfirst.frc.team4028.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;

public class TrajectoryDriveController extends Robot {
	
	//Trajectory _trajectory;
	double[][] _leftMotionProfile;
	double[][] _rightMotionProfile;
	TrajectoryFollower _leftFollower = new TrajectoryFollower("left");
	TrajectoryFollower _rightFollower = new TrajectoryFollower("right");
	double _direction;
	double _heading;
	double _kTurn = 0.0;  // Should be a constant
	boolean _enabled = false;
	double _leftPower;
	double _rightPower;
	double _angleDiff;
	
	public TrajectoryDriveController() {
		_leftFollower.configure(0.0,  0.0,  0.0,  0.7,  0.0);
		_rightFollower.configure(0.0,  0.0,  0.0,  0.7,  0.0);
	}
	
	public boolean onTarget() {
		return _leftFollower.isTrajectoryFinished();
	}
	
	public void loadProfile(double[][] leftProfile, double[][] rightProfile, double direction, double heading) {
		reset();
		_leftMotionProfile = leftProfile;
		_rightMotionProfile = rightProfile;
		this._direction = direction;
		this._heading = heading;
		_leftPower = 0.0;
		_rightPower = 0.0;
		_angleDiff = 0.0;
	}
	
	public void loadProfileNoReset(Trajectory leftProfile, Trajectory rightProfile) {
		_leftFollower.setTrajectory(leftProfile);
		_rightFollower.setTrajectory(rightProfile);
	}
	
	public void reset() {
		_leftFollower.reset();
		_rightFollower.reset();
	}
	
	public int getFollowerCurrentSegment() {
		return _leftFollower.getCurrentSegment();
	}
	
	public int getNumSegments() {
		return _leftFollower.getNumSegments();
	}
	
	public double[] update(double leftEncoderPosition, double rightEncoderPosition, double headingInDegrees, int currentSegment) {
		if (!_enabled) {
			DriverStation.reportError("Not Enabled", false);
			return new double[] {0, 0};
		}
		
		if (onTarget()) {
			DriverStation.reportError("At Target", false);
			return new double[] {0, 0};
		} else {
			double distanceL = _direction * leftEncoderPosition;
			double distanceR = _direction * rightEncoderPosition;
			
			_leftPower = _direction * _leftFollower.calculate(distanceL, _leftMotionProfile, currentSegment);
			_rightPower = _direction * _rightFollower.calculate(distanceR, _rightMotionProfile, currentSegment);
			
			double goalHeading = _leftFollower.getHeading();
			double goalHeadingInDegrees = BeefyMath.arctan(goalHeading);
			double observedHeading = headingInDegrees;

			double turn = _kTurn * (observedHeading - goalHeadingInDegrees);
			
			return new double[] {_leftPower - turn, _rightPower + turn};
		}
	}
	
	public void enable() {
		_leftFollower.reset();
		_rightFollower.reset();
		_enabled = true;
	}
	
	public void disable() {
		_enabled = false;
	}
	
	public boolean isEnable() {
		return _enabled;
	}
	
	public int getCurrentSegment() {
		return _leftFollower.getCurrentSegment();
	}
	
	public double getCurrentHeading() {
		return _leftFollower.getHeading();
	}
	
	public double getAngleDiff() {
		return _angleDiff;
	}
}