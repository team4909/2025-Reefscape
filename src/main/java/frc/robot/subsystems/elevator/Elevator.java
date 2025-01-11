package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIO m_io;
    private final double L1Setpoint = 1;
    private final double L2Setpoint = 5;
    private final double L3Setpoint = 0;
    private final double L4Setpoint = 0;

    public Elevator(ElevatorIO io) {
        m_io = io;
    
      }
    
      public Command moveUp() {
        return this.run(() -> m_io.setVoltage(1));
      }
    
      public Command stop() {
        return this.run(() -> m_io.setVoltage(0));
      }
    
      public Command moveDown() {
        return this.run(() -> m_io.setVoltage(-1));
      }

      public Command goToL1(){
        return  this.run(() -> m_io.gotosetpoint(L1Setpoint));
      }
    
}
