package org.usfirst.frc.team4028.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

public class Trajectory {
	public int _segmentsLoaded = 0;
	public static class Pair {
		public Pair (Trajectory leftDrive, Trajectory rightDrive) {
			this.left = leftDrive;
			this.right = rightDrive;
		}
		
		public Trajectory left;
		public Trajectory right;
		
	}
	
	public static class Segment {
		public double _pos, _vel, _acc, _heading, _dt, _x, _y;
		
		public Segment() {
		}
		
		public Segment(double pos, double vel, double acc, double heading, double dt, double x, double y) {
			this._pos = pos;
			this._vel = vel;
			this._acc = acc;
			this._heading = heading;
			this._dt = dt;
			this._x = x;
			this._y = y;
		}
		
		public Segment(Segment to_copy) {
			_pos = to_copy._pos;
			_vel = to_copy._vel;
			_acc = to_copy._acc;
			_heading = to_copy._heading;
			_dt = to_copy._dt;
			_x = to_copy._x;
			_y = to_copy._y;
		}
		
		public String toString() {
			return "pos: " + _pos + "; vel: " + _vel + "; acc: " + _acc + "; heading: " + _heading;
		}
	}
	
	Segment[] _segments = null;
	boolean _isYInverted = false;
	
	public Trajectory(int length) {
		_segments = new Segment[length];
		for (int i = 0; i < length; ++i) {
			_segments[i] = new Segment();
		}
	}
	
	public Trajectory(int length, double[][] motionProfile) {
		_segments = new Segment[length];
		for (int i = 0; i < length; ++i) {
			_segments[i] = new Segment(motionProfile[i][0], motionProfile[i][1], 
					motionProfile[i][2], motionProfile[i][3], motionProfile[i][4], motionProfile[i][5], motionProfile[i][6]);
		}
	}
	
	public Trajectory(Segment[] segments) {
		_segments = segments;
	}
	
	public void setYInverted(boolean inverted) {
		_isYInverted = inverted;
	}
	
	public int getNumSegments() {
		return _segments.length;
	}
	
	public Segment getSegment(int index) {
		if (index < getNumSegments()) {
			if(!_isYInverted)
			{
				return _segments[index];
			} else {
				Segment segment = new Segment(_segments[index]);
				segment._y *= -1.0;
				// Ensure to convert heading as well
				return segment;
			}
		} else {
			return new Segment();
		}
	}
	
	public void setSegment(int index, Segment segment) {
		if (index < getNumSegments()) {
			_segments[index] = segment;
		}
	}
	
	public void printSegment(int index) {
		Segment segment = new Segment(_segments[index]);
		DriverStation.reportError(Double.toString(segment._pos), false);
	}
	
	public void printSegmentsLoaded() {
		DriverStation.reportError(Integer.toString(_segmentsLoaded), false);
	}
}
