package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public interface ElevatorIO {

    public static class ElevatorIOInputs { 
        public double voltage = 0d;
    }

    public default void setVoltage(double voltage) {}
    
    public default void setBrakeMode(boolean enableBrakeMode) {}

    public abstract Command gotosetpoint(double setpoint, double gearRatio);
}