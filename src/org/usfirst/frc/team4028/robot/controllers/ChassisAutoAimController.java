package org.usfirst.frc.team4028.robot.controllers;

import org.usfirst.frc.team4028.robot.util.PIDCalculator;
import org.usfirst.frc.team4028.robot.Robot;

import edu.wpi.first.wpilibj.DriverStation;

public class ChassisAutoAimController extends Robot{
	
	PIDCalculator _autoAimPID;
	double _kp = 0.11;
	double _ki = 0.0;
	double _kd = 0.0;
	double _deadband;
	double _heading;
	boolean _enabled = false;
	
	public ChassisAutoAimController() {
		_autoAimPID = new PIDCalculator(_kp, _ki, _kd);
	}
	
	public boolean onTarget() {
		return _autoAimPID.onTarget();
	}
	
	public void loadNewTarget(double angle) {
		_autoAimPID.reset();
		_autoAimPID.setSetpoint(angle);
		DriverStation.reportError("New Setpoint Loaded", false);
	}
	
	public double update(double heading) {
		double motorOutput = _autoAimPID.calculate(heading);
		return motorOutput;
	}
}