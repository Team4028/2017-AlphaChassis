package org.usfirst.frc.team4028.robot.util;

/* 
 * Holds the beefiest, and therefore the best, methods pertaining
 * to mathematical computations used on the robot. Partially inspired
 * by Beefulus
 * 
 *          (__)
 *        	(oo)                      
 *	 /-------\/                   
 *  / |     ||                                  
 *    ||----||
 *    ^^    ^^
 */

public class BeefyMath {
  public static double getDifferenceInAngleRadians(double from, double to) {
    return boundAngleNegPiToPiRadians(to - from);
  }
  
  public static double boundAngleNegPiToPiRadians(double angle) {
    while (angle >= Math.PI) {
      angle -= 2.0 * Math.PI;
    }
    while (angle < -Math.PI) {
      angle += 2.0 * Math.PI;
    }
    return angle;
  }
  
  public static double convertDegreesToRadians(double angle) {
	  return Math.toRadians(angle);
  }
  
  public static double arctan(double heading) {
	  double degrees = Math.toDegrees(Math.atan(heading));
	  return degrees;
  }
}