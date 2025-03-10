package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    
    @AutoLog
    public static class ElevatorIOInputs { 
        public boolean frontMotorConnected = false;
        public boolean backMotorConnected = false;
        
        public double frontMotorVoltage = 0.0;
        public double backMotorVoltage = 0.0;

        public double frontMotorPosition = 0.0;
        public double backMotorPosition = 0.0;

        public double goalPosition = 0.0;

        public double heightInch = 0.0;

        public double velocity = 0.0;
    }

    public abstract void updateInputs (ElevatorIOInputsAutoLogged m_inputs) ;

    public default void setVoltage(double voltage) {}
    
    public default void setBrakeMode(boolean enableBrakeMode) {}

    public default void gotosetpoint(double setpoint, double gearRatio) {}

    public default void setPosition(double position) {}
    
    public default double getSetpoint() { return 0; }


}