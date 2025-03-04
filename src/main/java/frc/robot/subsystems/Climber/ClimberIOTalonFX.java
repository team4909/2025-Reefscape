package frc.robot.subsystems.Climber;

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
import edu.wpi.first.units.measure.Voltage;

public class ClimberIOTalonFX implements ClimberIO {
  private final TalonFX m_Climbermotor;

  

  private double m_rotations;
  final PositionVoltage m_request;
  

  public ClimberIOTalonFX() {
    m_Climbermotor = new TalonFX(25, "CANivore2");

    m_request = new PositionVoltage(0).withSlot(0);

    final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();
    final MotorOutputConfigs outputConfigs = new MotorOutputConfigs();

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
        m_rotations = rotations;
        m_Climbermotor.setControl(m_request.withPosition(rotations));
    }

  public double getVelocity(){
    return m_Climbermotor.getVelocity().getValueAsDouble();
}
public double getVoltage(){
    return m_Climbermotor.getMotorVoltage().getValueAsDouble();
}

public double getPosition() {
    return m_Climbermotor.getPosition().getValueAsDouble();
}

public double getSetpoint(){
    return m_rotations;
}

public void setPosition(double position){
  m_Climbermotor.setPosition(position);
}

}
