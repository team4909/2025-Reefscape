package frc.robot.subsystems.algae;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeIO {
    @AutoLog
    public static class AlgaeIOInputs {
        public double speed = 0.0;
      }
    
      public default void setSpeed(double speed) {}
    
      public default void setBrakeMode(boolean enableBrakeMode) {}
    }
