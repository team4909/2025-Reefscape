package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX m_shootermotor;

  public ShooterIOTalonFX() {
    m_shootermotor = new TalonFX(20, "CANivore2");

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

  public void updateInputs(ShooterIOInputsAutoLogged m_inputs) {
    m_inputs.speed = m_shootermotor.getVelocity().getValueAsDouble();
    m_inputs.statorCurrent = m_shootermotor.getStatorCurrent().getValueAsDouble();
    m_inputs.supplyCurrent = m_shootermotor.getSupplyCurrent().getValueAsDouble();
        // double motorRPS = m_back.getVelocity().getValueAsDouble();
        // m_inputs.elevatorRPM = motorRPS*60;
        // m_inputs.heightInch = m_back.getPosition().getValueAsDouble() / m_gearRatio;
        // m_inputs.setpointInch = m_rotations / m_gearRatio;
    }
  
}
