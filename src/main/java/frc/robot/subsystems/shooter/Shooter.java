package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    super("Shooter");
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
    m_io.updateInputs(m_inputs);
    // there is probably a better way to do this
    if (this.getCurrentCommand() != null) {
      m_inputs.currentCommand = this.getCurrentCommand().getName();
    } else {
      m_inputs.currentCommand = "null";
    }
    Logger.processInputs(this.getName(), m_inputs);

  }

  public Command setDefaultDoNotRun() {
    return this.run(() -> this.setDefaultCommand(this.stop()));
  }

  public Command setDefaultRunToCurrentSpike() {
    return this.run(() -> this.setDefaultCommand(this.runToCurrentSpike())); //@todo
  }

  public Command runToCurrentSpike() {
    return this.run(() -> m_io.setSpeed(0.2)).until(() -> m_inputs.statorCurrent > 15).andThen(this.stop()).withName("RunToCurrentSpike");
  }
}
