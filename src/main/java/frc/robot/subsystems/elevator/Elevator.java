package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class Elevator extends SubsystemBase {
    private final ElevatorIO m_io;

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
    
}
