package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
    
    @AutoLog
    public static class AlgaeIOInputs { 
        public double shooterVoltage = 0d;
        public double shooterCurrent = 0;
        public double shootVelocity = 0;
        public double wristPosition = 0;
        public double wristSetpoint = 0;
    }

    public default void setShootVoltage(double voltage) {}

    public default void setPivotVoltage(double voltage) {}
    
    public default void setBrakeMode(boolean enableBrakeMode) {}

    public default void gotosetpoint(double setpoint, double gearRatio) {}

    public default void setPosition(double position) {}

	public void updateInputs(AlgaeIOInputs m_inputs);

	public void holdShooterPos();
}