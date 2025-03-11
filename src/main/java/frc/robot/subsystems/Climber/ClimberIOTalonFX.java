package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX m_Climbermotor;

  private StatusSignal<AngularVelocity> m_velocity;
  private StatusSignal<Current> m_supplyCurrent;
  private StatusSignal<Voltage> m_motorVoltage;
  private StatusSignal<Angle> m_position;

  final PositionVoltage m_request;
  

  public ClimberIOTalonFX() {
    m_Climbermotor = new TalonFX(25, "CANivore2");

    m_request = new PositionVoltage(0).withSlot(0);

    final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();
    final MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

    m_velocity = m_Climbermotor.getVelocity();
    m_supplyCurrent = m_Climbermotor.getSupplyCurrent();
    m_motorVoltage = m_Climbermotor.getMotorVoltage();
    m_position = m_Climbermotor.getPosition();

    outputConfigs.NeutralMode = NeutralModeValue.Brake;
    climberMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    climberMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.5; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output
        slot0Configs.kG = 0;

    m_Climbermotor.getConfigurator().apply(climberMotorConfig);
    m_Climbermotor.getConfigurator().apply(outputConfigs);
    m_Climbermotor.getConfigurator().apply(slot0Configs);

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

  
public void gotosetpoint(double setpoint, double gearRatio) {
    double rotations = setpoint * gearRatio;
    m_Climbermotor.setControl(m_request.withPosition(rotations));
}

public void setPosition(double position){
  m_Climbermotor.setPosition(position);
}

@Override
public void updateInputs(ClimberIOinputs inputs) {
  inputs.motorConnected = BaseStatusSignal.refreshAll(m_velocity, m_supplyCurrent, m_motorVoltage, m_position).isOK();

  inputs.velocity = m_velocity.getValueAsDouble();

  inputs.motorSupplyCurrent = m_supplyCurrent.getValueAsDouble();
  inputs.motorVoltage = m_motorVoltage.getValueAsDouble();

  inputs.motorPosition = m_position.getValueAsDouble();
  inputs.motorSetpoint = m_request.getPositionMeasure().magnitude();
}
}
