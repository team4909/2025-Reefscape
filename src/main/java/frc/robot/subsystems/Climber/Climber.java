package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climber.ClimberIO.ClimberIOinputs;

public class Climber extends SubsystemBase {
  private ClimberIO m_io;
  private final ClimberIOinputsAutoLogged m_inputs = new ClimberIOinputsAutoLogged();

  private final double climb = -190.866;

  final double m_gearRatio = 1d;

  public Climber(ClimberIO io) {
    m_io = io;
  }

  public Command climb() {
    return this.run(() -> m_io.setSpeed(-1));
  }

  public Command stop() {
    return this.run(() -> m_io.setSpeed(0));
  }

  public Command lower() {
    return this.run(() -> m_io.setSpeed(1));
  }

  public Command climbPosition() {
    return this.runOnce(() -> {
      m_io.gotosetpoint(climb, m_gearRatio);
    });
  }

  public Command reZero(){
    return this.runOnce(() -> {
        m_io.setPosition(0.0);
    }).withName("ReZero");
  }

  @Override
      public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs(getName()+"/", m_inputs);
      }
    
}
