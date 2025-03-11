package frc.robot.subsystems.algae;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIOTalonFX extends SubsystemBase implements AlgaeIO {

    private final TalonFX m_shootMotor;
    private final TalonFX m_pivotMotor;

    final PositionVoltage m_request;

    private StatusSignal<Voltage> m_shooterVoltage;
    private StatusSignal<Current> m_shooterCurrent;
    private StatusSignal<AngularVelocity> m_shooterVelocity;

    private StatusSignal<Angle> m_wristPosition;
    private StatusSignal<AngularVelocity> m_wristVelocity;
    private StatusSignal<Voltage> m_wristVoltage;
    private StatusSignal<Current> m_wristCurrent;

    public AlgaeIOTalonFX() {
        m_shootMotor = new TalonFX(23, "CANivore2");
        m_pivotMotor = new TalonFX(24, "CANivore2");

        m_request = new PositionVoltage(0).withSlot(0);

        final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

        motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // in init function, set slot 0 gains

        motorConfig.Slot0.kP = 0.5; // An error of 1 rotation results in 2.4 V output
        motorConfig.Slot0.kI = 0; // no output for integrated error
        motorConfig.Slot0.kD = 0; // A velocity of 1 rps results in 0.1 V output
        motorConfig.Slot0.kG = 0;

        m_pivotMotor.setPosition(0);
        m_shootMotor.getConfigurator().apply(motorConfig);
        m_pivotMotor.getConfigurator().apply(motorConfig);
    }

    public void setShootVoltage(double voltage) {
        final VoltageOut request = new VoltageOut(0);
        m_shootMotor.setControl(request.withOutput(voltage));
    }

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
        m_pivotMotor.setControl(m_request.withPosition(rotations));
    }

    public void setPosition(double position) {
        m_pivotMotor.setPosition(position);
    }

    public void updateInputs(AlgaeIOInputs inputs) {
        inputs.shooterConnected = m_shootMotor.getMotorVoltage().isOK();
        inputs.shooterVoltage = m_shootMotor.getMotorVoltage().getValueAsDouble();
        inputs.shooterCurrent = m_shootMotor.getSupplyCurrent().getValueAsDouble();
        inputs.shootVelocity = m_shootMotor.getVelocity().getValueAsDouble();
        inputs.wristPosition = m_pivotMotor.getPosition().getValueAsDouble();
        inputs.wristSetpoint = m_rotations;
    }

    public void holdShooterPos() {
        m_shootMotor.setControl(
                m_request.withPosition(m_shootMotor.getPosition().getValueAsDouble()));
    }
}
