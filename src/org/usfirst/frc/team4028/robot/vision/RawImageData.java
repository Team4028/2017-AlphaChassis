package org.usfirst.frc.team4028.robot.vision;

/* 
 * DTO (data transfer object) This class represents the data retrieved from the RoboRealm API
 */
public class RawImageData 
{
	public long Timestamp;
	
    public double ResponseTimeMSec;

    public int BlobCount;

    public int NorthWestX;
    public int NorthWestY;

    public int NorthEastX;
    public int NorthEastY;

    public int SouthWestX;
    public int SouthWestY;

    public int SouthEastX;
    public int SouthEastY;

    public Dimension FOVDimensions;

}
