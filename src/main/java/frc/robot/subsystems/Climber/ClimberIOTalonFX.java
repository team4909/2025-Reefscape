package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX m_Climbermotor;

  public ClimberIOTalonFX() {
    m_Climbermotor = new TalonFX(20, "CANivore2");

    final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();
    climberMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    climberMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_Climbermotor.getConfigurator().apply(climberMotorConfig);
  }

  @Override
  public void setSpeed(double speed) {
    m_Climbermotor.setControl(new DutyCycleOut(speed));
  }

   @Override
  public void setBrakeMode(boolean enableBrakeMode) {
    final NeutralModeValue neutralModeValue =
        enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        m_Climbermotor.setNeutralMode(neutralModeValue);
  }
    
}
