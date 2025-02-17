package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable elevatorTable = inst.getTable("Elevator");
    private final DoublePublisher  motorValPub= elevatorTable.getDoubleTopic("Motor Val").publish();
    private final DoublePublisher  motorVolPub= elevatorTable.getDoubleTopic("Motor Vol").publish();
    private final DoublePublisher  rotPub= elevatorTable.getDoubleTopic("Rotations").publish();
    private final DoublePublisher  setPub= elevatorTable.getDoubleTopic("Setpoint").publish();

    private final ElevatorIO m_io;
    private final double L1Setpoint = 29;
    private final double L2Setpoint = 38.5;//32.5
    private final double L3Setpoint = 51.5;
    private final double L4Setpoint = 77; // 74.5 and 78 dp on feb 1
    private final double L2ASetpoint = 48.5; 
    private final double L3ASetpoint = 64.5;
    // private final double L1Setpoint = 1;
    // private final double L2Setpoint = 2;//32.5
    // private final double L3Setpoint = 3;
    // private final double L4Setpoint = 4; // 74.5 and 78 dp on feb 1
    // private final double L2ASetpoint = 2.5; 
    // private final double L3ASetpoint = 3.5
    
    ;
    //inch to rotations of the motor
    final double m_gearRatio = 0.5 * (1d / (1.75100 * Math.PI)) * ( 2d / 3d ) * 25;

    public Elevator(ElevatorIO io) {
        m_io = io;
    
      }
    
      public Command moveUp() {
        return this.runOnce(() -> m_io.setVoltage(3)).withName("UP");
      }
    
      public Command stop() {
        return this.runOnce(() -> m_io.setVoltage(0)).withName("Stop");
      }
    
      public Command moveDown() {
        return this.runOnce(() -> m_io.setVoltage(-3)).withName("Move Down");
      }

      public Command goToL1(){
        // return  this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
        return this.runOnce(() -> {
            m_io.gotosetpoint(L1Setpoint, m_gearRatio);
        }).withName("L1");
      }
      public Command goToL2(){
        // return  this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
        return this.runOnce(() -> {
            m_io.gotosetpoint(L2Setpoint, m_gearRatio);
        }).withName("L2");
      }
      public Command goToL3(){
        // return  this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
        return this.runOnce(() -> {
            m_io.gotosetpoint(L3Setpoint, m_gearRatio);
        }).withName("L3");
      }
      public Command goToL4(){
        // return  this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
        return this.runOnce(() -> {
            m_io.gotosetpoint(L4Setpoint, m_gearRatio);
        }).withName("L4");
      }
      public Command goToL3A(){
        // return  this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
        return this.runOnce(() -> {
            m_io.gotosetpoint(L3ASetpoint, m_gearRatio);
        }).withName("L3A");
      }
      public Command goToL2A(){
        // return  this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
        return this.runOnce(() -> {
            m_io.gotosetpoint(L2ASetpoint, m_gearRatio);
        }).withName("L2A");
      }
      public Command goUpInch(){
        return this.runOnce(()-> {
          m_io.gotosetpoint(m_io.getPosition()+(1/10), m_gearRatio);
        }).withName("Inch");
      }
      public Command reZero(){
        return this.runOnce(() -> {
            m_io.setPosition(L1Setpoint*m_gearRatio);
        }).withName("ReZero");
      }

      public Command testPrint(){
        return this.runOnce(() -> {
            System.out.println("Test");
        }).withName("Test");
      }
      @Override
      public void periodic() {
        super.periodic();
        motorValPub.set(m_io.getVelocity());
        motorVolPub.set(m_io.getVoltage());
        setPub.set(m_io.getSetpoint());
        rotPub.set(m_io.getPosition());
      }
}
