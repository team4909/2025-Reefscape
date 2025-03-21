package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    super("Shooter");
    m_io = io;
    // setDefaultCommand(new ConditionalCommand(this.stop(), this.slowShoot(), () -> m_inputs.statorCurrent > 15));
  }

  public Command slowShoot() {
    return this.run(() -> m_io.setSpeed(-0.2)).withName("SlowShoot");
  }

  public Command shoot() {
    return this.run(() -> m_io.setSpeed(-0.3)).withName("Shoot");
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
    Logger.processInputs(this.getName(), m_inputs);
    if (this.getCurrentCommand() != null) {
      SmartDashboard.putString("shooter/command", this.getCurrentCommand().getName());
    } else {
      SmartDashboard.putString("shooter/command", "null");
    }
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
