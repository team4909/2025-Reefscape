package frc.robot.subsystems.Climber;


public interface ClimberIO {
  
public static class ClimberIOinputs{
  public double speed = 0.0;
}

public default void setSpeed(double speed) {}

public default void setBrakeMode(boolean enableBrakeMode) {}
}