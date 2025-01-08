package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOTalonFX implements ElevatorIO {

    private final TalonFX m_left;
    private final TalonFX m_right;

    public ElevatorIOTalonFX() {
    
        m_left = new TalonFX(24, "CANivore1");
        m_right = new TalonFX(25, "CANivore1");

        final TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        m_left.getConfigurator().apply(elevatorMotorConfig);
        m_right.getConfigurator().apply(elevatorMotorConfig);
    }

    public void setVoltage(double voltage) {
        final VoltageOut m_request = new VoltageOut(0);

        m_left.setControl(m_request.withOutput(-voltage));
        m_right.setControl(m_request.withOutput(voltage));
    }
    
    public void setBrakeMode(boolean enableBrakeMode) {
        final NeutralModeValue neutralModeValue =
            enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        m_left.setNeutralMode(neutralModeValue);
        m_right.setNeutralMode(neutralModeValue);
    }
}
