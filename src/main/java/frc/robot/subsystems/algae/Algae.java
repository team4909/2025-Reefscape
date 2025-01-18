package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Algae extends SubsystemBase {
    private final AlgaeIO m_io;

    public Algae(AlgaeIO io) {
        m_io = io;
      }

    public Command shootAlgae() {
        return this.run(() -> m_io.setSpeed(-0.1));
  }

    public Command stop() {
        return this.run(() -> m_io.setSpeed(0));
  }

    public Command intake() {
        return this.run(() -> m_io.setSpeed(1));
  }
}

