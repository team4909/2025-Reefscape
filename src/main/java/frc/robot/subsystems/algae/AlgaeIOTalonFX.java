package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class AlgaeIOTalonFX implements AlgaeIO {
    
     private final TalonFX m_algaeMotor;

     public AlgaeIOTalonFX (){
        m_algaeMotor = new TalonFX(23, "CANivore2");

        final TalonFXConfiguration algaeMotorConfig = new TalonFXConfiguration();
        algaeMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        algaeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_algaeMotor.getConfigurator().apply(algaeMotorConfig);
     }

     @Override
    public void setSpeed(double speed) {
        m_algaeMotor.setControl(new DutyCycleOut(speed));
  }

    @Override
    public void setBrakeMode(boolean enableBrakeMode) {
    final NeutralModeValue neutralModeValue =
        enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_algaeMotor.setNeutralMode(neutralModeValue);
  }
}
