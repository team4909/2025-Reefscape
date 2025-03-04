package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO m_io;

  public Shooter(ShooterIO io) {
    m_io = io;
  }

  public Command shoot() {
    return this.run(() -> m_io.setSpeed(-0.2));
  }

  public Command shootTrough() {
    return this.run(() -> m_io.setSpeed(-0.3));
  }

  public Command stop() {
    return this.run(() -> m_io.setSpeed(0));
  }
  public Command stopInstant() {
    return this.runOnce(() -> m_io.setSpeed(0));
  }

  public Command intake() {
    return this.run(() -> m_io.setSpeed(1));
  }
}
