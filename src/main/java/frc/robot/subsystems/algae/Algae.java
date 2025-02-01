package frc.robot.subsystems.algae;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private final NetworkTable algaeTable = inst.getTable("Algae");
    private final DoublePublisher  motorValPub= algaeTable.getDoubleTopic("Motor Val").publish();
    private final DoublePublisher  motorVolPub= algaeTable.getDoubleTopic("Motor Vol").publish();
    private final DoublePublisher  rotPub= algaeTable.getDoubleTopic("Rotations").publish();
    private final DoublePublisher  setPub= algaeTable.getDoubleTopic("Setpoint").publish();

    private final AlgaeIO m_io;
    private final double DownPosition = 0;
    private final double ExtendedPosition = -1.5;//32.5
    //inch to rotations of the motor
    final double m_gearRatio = 1d;

    public Algae(AlgaeIO io) {
        m_io = io;
        // setDefaultCommand(stop());
      }
    
      public Command moveUp() {
        return this.runOnce(() -> m_io.setVoltage(1)).withName("UP");
      }
    
      public Command stop() {
        return this.runOnce(() -> m_io.setVoltage(0)).withName("Stop");
      }
    
      public Command moveDown() {
        return this.runOnce(() -> m_io.setVoltage(-1)).withName("Move Down");
      }

      public Command down(){
        // return  this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
        return this.runOnce(() -> {
            m_io.gotosetpoint(DownPosition, m_gearRatio);
        }).withName("L1");
      }
      public Command extend(){
        // return  this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
        return this.runOnce(() -> {
            m_io.gotosetpoint(ExtendedPosition, m_gearRatio);
        }).withName("L2");
      }
      public Command reZero(){
        return this.runOnce(() -> {
            m_io.setPosition(DownPosition*m_gearRatio);
        }).withName("ReZero");
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
