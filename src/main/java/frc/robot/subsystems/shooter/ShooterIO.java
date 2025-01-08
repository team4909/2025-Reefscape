package frc.robot.subsystems.shooter;

public interface ShooterIO {

  public static class ShooterIOInputs {
    public double speed = 0.0;
  }

  public default void setSpeed(double speed) {}

  public default void setBrakeMode(boolean enableBrakeMode) {}
}
