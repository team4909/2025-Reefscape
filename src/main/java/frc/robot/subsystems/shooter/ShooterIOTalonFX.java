package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;

public class ShooterIOTalonFX implements ShooterIO {

  private final TalonFX m_shootermotor;

  private StatusSignal<AngularVelocity> m_velocity;
  private StatusSignal<Current> m_statorCurrent, m_supplyCurrent;
  private StatusSignal<Voltage> m_motorVoltage;


  public ShooterIOTalonFX() {
    m_shootermotor = new TalonFX(20, "CANivore2");

    m_velocity = m_shootermotor.getVelocity();
    m_statorCurrent = m_shootermotor.getStatorCurrent();
    m_supplyCurrent = m_shootermotor.getSupplyCurrent();
    m_motorVoltage = m_shootermotor.getMotorVoltage();

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
    m_inputs.motorConnected = BaseStatusSignal.refreshAll(m_velocity, m_statorCurrent, m_supplyCurrent, m_motorVoltage).isOK();

    m_inputs.velocity = m_velocity.getValueAsDouble();

    m_inputs.statorCurrent = m_statorCurrent.getValueAsDouble();
    m_inputs.supplyCurrent = m_supplyCurrent.getValueAsDouble();

    m_inputs.voltage = m_motorVoltage.getValueAsDouble();
    }
  
}
