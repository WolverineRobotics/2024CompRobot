package frc.robot;

import edu.wpi.first.math.util.Units;

// Base class to store all data related to april tags
public class AprilTagData {

    private int id, rot; 
    private double x, y, z;

    public int getId(){ return id; }
    public double getX(){ return x; }
    public double getY(){ return y; }
    public double getZ(){ return z; }
    public int getRot(){ return rot; }
    
    public int getHeading(){     
        // No need to check if rot is past 360deg, the values of all needed declared variables
        // Of this type never exceed 359deg
        int heading = rot;
        if (heading > 180) {
            heading = heading - 360;
        }
        return heading;
    }

    public AprilTagData(int _id, double _x, double _y, double _z, int _rot){
        id = _id;
        x = _x;
        y = _y;
        x = _x;
        rot = _rot;
    }

    
    /* Contains the positions of all the AprilTags on the CRESCENDO game field */
    // List index = id - 1
    
    public static final AprilTagData[] april_tags = new AprilTagData[]{
        new AprilTagData(1,  
            Units.inchesToMeters( 593.68 ),          //X 
            Units.inchesToMeters( 9.68 ),          //Y
            Units.inchesToMeters( 53.38 ), 120), //Z, rotation 

        new AprilTagData(2,  
            Units.inchesToMeters( 637.21), 
            Units.inchesToMeters( 34.79), 
            Units.inchesToMeters( 53.38), 120),

        new AprilTagData(3,  
            Units.inchesToMeters( 652.73), 
            Units.inchesToMeters( 196.17), 
            Units.inchesToMeters( 57.13), 180),

        new AprilTagData(4,  
            Units.inchesToMeters( 652.73), 
            Units.inchesToMeters( 218.42), 
            Units.inchesToMeters( 57.13), 180),

        new AprilTagData(5,  
            Units.inchesToMeters( 578.77), 
            Units.inchesToMeters( 323), 
            Units.inchesToMeters( 53.38), 270),

        new AprilTagData(6,  
            Units.inchesToMeters( 72.5), 
            Units.inchesToMeters( 323), 
            Units.inchesToMeters( 53.38), 270),

        new AprilTagData(7,  
            Units.inchesToMeters( -1.50), 
            Units.inchesToMeters( 218.42), 
            Units.inchesToMeters( 57.13), 0),

        new AprilTagData(8,  
            Units.inchesToMeters( -1.50), 
            Units.inchesToMeters( 196.17), 
            Units.inchesToMeters( 57.13), 0),

        new AprilTagData(9,  
            Units.inchesToMeters( 14.02), 
            Units.inchesToMeters( 34.79), 
            Units.inchesToMeters( 53.38), 60),

        new AprilTagData(10, 
            Units.inchesToMeters( 57.54), 
            Units.inchesToMeters( 9.68), 
            Units.inchesToMeters( 53.38), 60),

        new AprilTagData(11, 
            Units.inchesToMeters( 468.69), 
            Units.inchesToMeters( 146.19), 
            Units.inchesToMeters( 52.00), 300),

        new AprilTagData(12, 
            Units.inchesToMeters( 468.69), 
            Units.inchesToMeters( 177.10), 
            Units.inchesToMeters( 52.00), 60),

        new AprilTagData(13, 
            Units.inchesToMeters( 441.74), 
            Units.inchesToMeters( 161.62), 
            Units.inchesToMeters( 52.00), 180),

        new AprilTagData(14, 
            Units.inchesToMeters( 209.48), 
            Units.inchesToMeters( 161.62), 
            Units.inchesToMeters( 52.00), 0),

        new AprilTagData(15, 
            Units.inchesToMeters( 182.73), 
            Units.inchesToMeters( 177.10), 
            Units.inchesToMeters( 52.00), 120),

        new AprilTagData(16, 
            Units.inchesToMeters( 182.73), 
            Units.inchesToMeters( 146.19), 
            Units.inchesToMeters( 52.00), 240)
        
    };
}
