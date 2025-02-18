package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX m_Climbermotor;

  public ClimberIOTalonFX() {
    m_Climbermotor = new TalonFX(20, "CANivore2");

    final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();
    final MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    climberMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    climberMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_Climbermotor.getConfigurator().apply(climberMotorConfig);
    m_Climbermotor.getConfigurator().apply(outputConfigs);
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
