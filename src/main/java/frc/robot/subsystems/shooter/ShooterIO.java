package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double velocity = 0.0;

    public double statorCurrent = 0.0;
    public double supplyCurrent = 0.0;

    public double voltage = 0.0;

    public boolean motorConnected = false;
  }

  public default void setSpeed(double speed) {}

  public default void setBrakeMode(boolean enableBrakeMode) {}

  public default void updateInputs(ShooterIOInputs inputs) {}
}
