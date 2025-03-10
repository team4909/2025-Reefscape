package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double speed = 0.0;
    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;
    public String currentCommand = "null";
  }

  public default void setSpeed(double speed) {}

  public default void setBrakeMode(boolean enableBrakeMode) {}

  public void updateInputs(ShooterIOInputsAutoLogged m_inputs);
}
