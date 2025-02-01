package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public interface AlgaeIO {
    
    @AutoLog
    public static class AlgaeIOInputs { 
        public double voltage = 0d;
    }

    public default void setVoltage(double voltage) {}
    
    public default void setBrakeMode(boolean enableBrakeMode) {}

    public default void gotosetpoint(double setpoint, double gearRatio) {}

    public default void setPosition(double position) {}

    public default double getVelocity() { return 0; }

    public default double getVoltage() { return 0; }

    public default double getPosition() { return 0; }

    public default double getSetpoint() { return 0; }
}