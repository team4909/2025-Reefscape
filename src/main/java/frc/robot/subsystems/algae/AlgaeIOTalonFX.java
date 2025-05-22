package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIOTalonFX extends SubsystemBase implements AlgaeIO {

    private final TalonFX m_shootMotor;
    private final TalonFX m_pivotMotor;
    private double m_rotations;
    final PositionVoltage m_request;
    // final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    // private final PositionVoltage m_request;

    public AlgaeIOTalonFX() {

        m_shootMotor = new TalonFX(23);
        m_pivotMotor = new TalonFX(24);

        m_request = new PositionVoltage(0).withSlot(0);

        final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        final TalonFXConfiguration shootMotorConfig = new TalonFXConfiguration();

        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        shootMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shootMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shootMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        shootMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // in init function, set slot 0 gains

        pivotMotorConfig.Slot0.kP = 0.5; // An error of 1 rotation results in 2.4 V output
        pivotMotorConfig.Slot0.kI = 0; // no output for integrated error
        pivotMotorConfig.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output
        pivotMotorConfig.Slot0.kG = 0;

        shootMotorConfig.Slot1.kP = 1; // An error of 1 rotation results in 2.4 V output
        shootMotorConfig.Slot1.kI = 0; // no output for integrated error
        shootMotorConfig.Slot1.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
        shootMotorConfig.Slot1.kG = 0;

        m_pivotMotor.setPosition(0);
        m_shootMotor.getConfigurator().apply(shootMotorConfig);
        m_pivotMotor.getConfigurator().apply(pivotMotorConfig);
    }

    public void setShootVoltage(double voltage) {
        final VoltageOut request = new VoltageOut(0);
        m_shootMotor.setControl(request.withOutput(voltage));
    }

    @Override
    public void setPivotVoltage(double voltage) {
        final VoltageOut request = new VoltageOut(0);
        m_pivotMotor.setControl(request.withOutput(voltage));
    }

    public void setBrakeMode(boolean enableBrakeMode) {
        final NeutralModeValue neutralModeValue = enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        m_shootMotor.setNeutralMode(neutralModeValue);
    }

    @Override
    public void gotosetpoint(double setpoint, double gearRatio) {
        double rotations = setpoint * gearRatio;
        m_rotations = rotations;
        m_pivotMotor.setControl(m_request.withPosition(rotations));
    }
    
    
    public void gotosetpointslow(double setpoint, double gearRatio, double voltage) {
        double rotations = setpoint * gearRatio;
        m_rotations = rotations;
        final VoltageOut request = new VoltageOut(0);
        m_pivotMotor.setControl(request.withOutput(voltage));
        m_pivotMotor.setControl(m_request.withPosition(rotations));
        
    }

    public void setPosition(double position) {
        m_pivotMotor.setPosition(position);
    }

    public void updateInputs(AlgaeIOInputsAutoLogged m_inputs) {
        m_inputs.shooterVoltage = m_shootMotor.getMotorVoltage().getValueAsDouble();
        m_inputs.shooterCurrent = m_shootMotor.getSupplyCurrent().getValueAsDouble();
        m_inputs.shootVelocity = m_shootMotor.getVelocity().getValueAsDouble();
        m_inputs.wristPosition = m_pivotMotor.getPosition().getValueAsDouble();
        m_inputs.wristSetpoint = m_rotations;
    }

    public void holdShooterPos() {
        m_shootMotor.setControl(
                m_request.withPosition(m_shootMotor.getPosition().getValueAsDouble()));
    }
}
