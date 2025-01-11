package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorIOTalonFX implements ElevatorIO {


    private final TalonFX m_left;
    private final TalonFX m_right;

    private final PositionVoltage m_request;

    public ElevatorIOTalonFX() {
    
        m_left = new TalonFX(21, "CANivore2");
        m_right = new TalonFX(22, "CANivore2");

        final double m_gearRatio = 5/1 * 5/1 * 72/48;
        final TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 2.4; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
        m_left.setControl(new Follower(22, false));

        m_right.setPosition(0);
        // create a position closed-loop request, voltage output, slot 0 configs
        m_request = new PositionVoltage(0).withSlot(0);

        // set position to 10 rotations
        
        m_left.getConfigurator().apply(slot0Configs);
        m_right.getConfigurator().apply(slot0Configs);

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

    public void gotosetpoint(double setpoint) {
        m_left.setControl(m_request.withPosition(setpoint));
        m_right.setControl(m_request.withPosition(setpoint));
    }
        
 
}
