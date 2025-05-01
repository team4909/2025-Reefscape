package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

	@AutoLog
	public static class ClimberIOinputs {
		public double velocity = 0.0;
		public double motorPosition = 0.0;
		public double motorSetpoint = 0.0;
		public double motorSupplyCurrent = 0.0;
		public double motorVoltage = 0.0;
		public boolean motorConnected = false;
	}

	public void setSpeed(double speed);

	public void setBrakeMode(boolean enableBrakeMode);

	public void updateInputs(ClimberIOinputs inputs);

	public void gotosetpoint(double setpoint, double gearRatio);

	public void setPosition(double position);
}