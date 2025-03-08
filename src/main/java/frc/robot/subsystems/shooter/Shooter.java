package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO m_io;

  public Shooter(ShooterIO io) {
    m_io = io;
  }

  public Command shoot() {
    return this.run(() -> m_io.setSpeed(-0.2)).withName("Shoot");
  }

  public Command shootTrough() {
    return this.run(() -> m_io.setSpeed(-0.3)).withName("ShootTrough");
  }

  public Command stop() {
    return this.run(() -> m_io.setSpeed(0)).withName("Stop");
  }
  public Command stopInstant() {
    return this.runOnce(() -> m_io.setSpeed(0)).withName("StopInstant");
  }

  public Command intake() {
    return this.run(() -> m_io.setSpeed(1)).withName("Intake");
  }

  @Override
  public void periodic() {
    if (this.getCurrentCommand() != null) {
      SmartDashboard.putString("shooter/command", this.getCurrentCommand().getName());
    } else {
      SmartDashboard.putString("shooter/command", "null");
    }
  }
}
