package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    @AutoLog
    public static class ElevatorIOInputs { 
        public double voltage = 0d;
        public double elevatorRPM = 0;
        public double heightInch = 0;
        public double setpointInch = 0;
    }

    public abstract void updateInputs (ElevatorIOInputsAutoLogged m_inputs) ;

    public default void setVoltage(double voltage) {}
    
    public default void setBrakeMode(boolean enableBrakeMode) {}

    public default void gotosetpoint(double setpoint, double gearRatio) {}

    public default void setPosition(double position) {}

    public default double getVelocity() { return 0; }

    public default double getVoltage() { return 0; }

    public default double getPosition() { return 0; }

    public default double getSetpoint() { return 0; }


}