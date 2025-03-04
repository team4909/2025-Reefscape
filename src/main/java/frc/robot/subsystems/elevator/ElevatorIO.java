package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs;

public interface ElevatorIO {
    
    @AutoLog
    public static class ElevatorIOInputs { 
        public boolean rightMotorConnected = false;
        public boolean leftMotorConnected = false;
        
        public double rightMotorVoltage = 0.0;
        public double leftMotorVoltage = 0.0;

        public double motorPosition = 0.0;

        public double goalPosition = 0.0;

        public double velocity = 0.0;
    }

    public default void setVoltage(double voltage) {}
    
    public default void setBrakeMode(boolean enableBrakeMode) {}

    public default void gotosetpoint(double setpoint, double gearRatio) {}

    public default void setPosition(double position) {}

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default double getPosition() {return 0.0;}
}