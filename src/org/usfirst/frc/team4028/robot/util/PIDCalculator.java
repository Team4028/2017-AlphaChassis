package org.usfirst.frc.team4028.robot.util;

import edu.wpi.first.wpilibj.util.BoundaryException;

public class PIDCalculator {
	
	private double _p; // "proportional" term
	private double _i; // "integral" term
	private double _d; // "derivative" term
	private double _maximumOutput = 0.75; 
	private double _minimumOutput = -0.75;
	private double _maximumInput = 180.0;
	private double _minimumInput = -180.0;
	private boolean _isContinuous = false; // do the endpoints wrap around e.g. absolute encoder
	private double _prevError = 0.0;
	private double _totalError = 0.0;
	private double _setpoint = 0.0;
	private double _error = 0.0;
	private double _result = 0.0;
	private double _lastInput = Double.NaN;
	private double _deadband = 0.0;
	
	public PIDCalculator() {
	}
	
	public PIDCalculator(double Kp, double Ki, double Kd) {
		_p = Kp;
		_i = Ki;
		_d = Kd;
	}
	
	public double calculate (double input) {
		_lastInput = input;
		_error = _setpoint - input;
		if (_isContinuous) {
			if (Math.abs(_error) > (_maximumInput - _minimumInput) / 2) {
				if (_error > 0) {
					_error = _error - _maximumInput + _minimumInput;
				} else {
					_error = _error + _maximumInput - _minimumInput;
				}
			}
		}
		
		if ((_error * _p < _maximumOutput) && (_error *_p > _minimumOutput))
		{
			_totalError += _error;
		} else {
			_totalError = 0.0;
		}
		
		double proportionalError = Math.abs(_error) < _deadband ? 0 : _error;
		
		_result = (_p * proportionalError + _i * _totalError * _d * (_error - _prevError));
		
		if (_result > _maximumOutput) {
            _result = _maximumOutput;
        } else if (_result < _minimumOutput) {
            _result = _minimumOutput;
        }
        return _result;  
	}
	
	public void setPID(double p, double i, double d) {
		_p = p;
		_i = i;
		_d = d;
	}
	
	public void setContinuous(boolean isContinuous) {
		_isContinuous = isContinuous;
	}
	
	public void setDeadband (double deadband) {
		_deadband = deadband;
	}
	
	public void setContinuous() {
		this._isContinuous = true;
	}
	
	public void setInputRange(double minimumInput, double maximumInput) {
        if (minimumInput > maximumInput) {
            throw new BoundaryException("Lower bound is greater than upper bound");
        }
        _minimumInput = minimumInput;
        _maximumInput = maximumInput;
        setSetpoint(_setpoint);
    }
	
	public void setOutputRange(double minimumOutput, double maximumOutput) 
	{
		if (minimumOutput > maximumOutput) {
			throw new BoundaryException("Lower bound is greater than upper bound");
		}
		
		_minimumOutput = minimumOutput;
		_maximumOutput = maximumOutput;
	}
	
	public void setSetpoint(double setpoint) {
        if (_maximumInput > _minimumInput) {
            if (setpoint > _maximumInput) {
                _setpoint = _maximumInput;
            } else if (setpoint < _minimumInput) {
                _setpoint = _minimumInput;
            } else {
                _setpoint = setpoint;
            }
        } else {
            _setpoint = setpoint;
        }
    }
	
	public boolean onTarget() {
        return _lastInput != Double.NaN && Math.abs(_lastInput - _setpoint) < _deadband;
    }

    /**
     * Reset all internal terms.
     */
    public void reset() {
        _lastInput = Double.NaN;
        _prevError = 0;
        _totalError = 0;
        _result = 0;
        _setpoint = 0;
    }

    public void resetIntegrator() {
        _totalError = 0;
    }

    public String getState() {
        String lState = "";

        lState += "Kp: " + _p + "\n";
        lState += "Ki: " + _i + "\n";
        lState += "Kd: " + _d + "\n";

        return lState;
    }

    public String getType() {
        return "PIDController";
    }
	
	public double getError() {
        return _error;
    }
	
	public double getSetpoint() {
		return _setpoint;
	}
	
	public double getP() {
		return _p;
	}
	
	public double getI() {
		return _i;
	}
	
	public double getD() {
		return _d;
	}
	
	public double get() {
		return _result;
	}
}
