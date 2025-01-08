package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX m_shootermotor;

  public ShooterIOTalonFX() {
    m_shootermotor = new TalonFX(20, "CANivore1");

    final TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();
    shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_shootermotor.getConfigurator().apply(shooterMotorConfig);
  }

  @Override
  public void setSpeed(double speed) {
    m_shootermotor.setControl(new DutyCycleOut(speed));
  }

  @Override
  public void setBrakeMode(boolean enableBrakeMode) {
    final NeutralModeValue neutralModeValue =
        enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    m_shootermotor.setNeutralMode(neutralModeValue);
  }
}
