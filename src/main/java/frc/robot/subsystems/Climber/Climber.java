package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final ClimberIO m_io;

  public Climber(ClimberIO io) {
    m_io = io;
  }

  public Command climb() {
    return this.run(() -> m_io.setSpeed(1));
  }

  public Command stop() {
    return this.run(() -> m_io.setSpeed(0));
  }

  public Command lower() {
    return this.run(() -> m_io.setSpeed(-1));
  }
    
}
