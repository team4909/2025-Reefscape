package frc.robot.subsystems.Climber;


public interface ClimberIO {
  
public static class ClimberIOinputs{
  public double velocity = 0.0;

  public double motorPosition = 0.0;
  
  public double motorVoltage = 0.0;
}

public default void setSpeed(double speed) {}

public default void setBrakeMode(boolean enableBrakeMode) {}

public default void updateInputs(ClimberIOinputs inputs) {}

public default void gotosetpoint(double setpoint, double gearRatio) {}

public default double getVelocity() { return 0; }

public default double getVoltage() { return 0; }

public default double getPosition() { return 0; }

public default double getSetpoint() { return 0; }

public default void setPosition(double position) {}
}