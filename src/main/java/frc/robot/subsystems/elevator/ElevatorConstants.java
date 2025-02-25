package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    
    //Gear Ratio (inch to rotations of the motor)
    //gear ratio is 2/3*25
    //spool diameter is 1.751
    public final static double kGEAR_RATIO = 0.5 * (1d / (1.75100 * Math.PI)) * ( 2d / 3d ) * 25;
    
    //Setpoints
    public final static double kL1 = 29;
    public final static double kL2 = 38.5;//32.5
    public final static double kL3 = 51.5;
    public final static double kL4 = 77; // 74.5 and 78 dp on feb 1
    public final static double kL2A = 48.5; 
    public final static double kL3A = 64.5;

    // private final double L1Setpoint = 1;
    // private final double L2Setpoint = 2;//32.5
    // private final double L3Setpoint = 3;
    // private final double L4Setpoint = 4; // 74.5 and 78 dp on feb 1
    // private final double L2ASetpoint = 2.5; 
    // private final double L3ASetpoint = 3.5
}
