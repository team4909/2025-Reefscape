package frc.robot.subsystems.elevator;

public interface ElevatorIO {

    public static class ElevatorIOInputs { 
        public double voltage = 0d;
    }

    public default void setVoltage(double voltage) {}
    
    public default void setBrakeMode(boolean enableBrakeMode) {}
}