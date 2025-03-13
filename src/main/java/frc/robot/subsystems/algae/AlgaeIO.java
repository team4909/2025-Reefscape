package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
    
    @AutoLog
    public static class AlgaeIOInputs { 
        public boolean shooterConnected = false;
        public double shooterVoltage = 0.0;
        public double shooterCurrent = 0.0;
        public double shootVelocity = 0.0;

        public boolean wristConnected = false;
        public double wristPosition = 0.0;
        public double wristSetpoint = 0.0;
        public double wristVoltage = 0.0;
        public double wristCurrent = 0.0;
    }

    public default void setShootVoltage(double voltage) {}

    public default void setPivotVoltage(double voltage) {}
    
    public default void setBrakeMode(boolean enableBrakeMode) {}

    public default void gotosetpoint(double setpoint, double gearRatio) {}

    public default void setPosition(double position) {}

	public void updateInputs(AlgaeIOInputs m_inputs);

	public void holdShooterPos();
}