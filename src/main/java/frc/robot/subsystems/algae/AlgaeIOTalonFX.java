package frc.robot.subsystems.algae;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeIOTalonFX extends SubsystemBase implements AlgaeIO{
    
    

    private final TalonFX m_right;
    private double m_rotations;
    final PositionVoltage m_request;
    //final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    //private final PositionVoltage m_request;

    public AlgaeIOTalonFX() {
        
        

        m_right = new TalonFX(23, "CANivore2");
        
        m_request = new PositionVoltage(0).withSlot(0);

        final TalonFXConfiguration algaeMotorConfig = new TalonFXConfiguration();
        final MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        outputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        outputConfigs.NeutralMode = NeutralModeValue.Brake;
        algaeMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        algaeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 5.5; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output
        slot0Configs.kG = 0.5;
     
        m_right.setPosition(0);
        m_right.getConfigurator().apply(algaeMotorConfig);
        m_right.getConfigurator().apply(outputConfigs);
        m_right.getConfigurator().apply(slot0Configs);
    }

    public void setVoltage(double voltage) {
        // return this.runOnce(()->{
            final VoltageOut request = new VoltageOut(0);
            m_right.setControl(request.withOutput(voltage));
        // });
        System.out.println("volts:" + m_right.getMotorVoltage());
    }
    
    public void setBrakeMode(boolean enableBrakeMode) {
        final NeutralModeValue neutralModeValue =
            enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        m_right.setNeutralMode(neutralModeValue);
    }
    @Override
    public void gotosetpoint(double setpoint, double gearRatio) {
        double rotations = setpoint * gearRatio;
        m_rotations = rotations;
        System.out.println("rotations:" + rotations);
        m_right.setControl(m_request.withPosition(rotations));

    }

    public double getVelocity(){
        return m_right.getVelocity().getValueAsDouble();
    }
    public double getVoltage(){
        return m_right.getMotorVoltage().getValueAsDouble();
    }
    @Override
    public double getPosition() {
        return m_right.getPosition().getValueAsDouble();
    }

    public double getSetpoint(){
        return m_rotations;
    }

    public void setPosition(double position){
        m_right.setPosition(position);
    }
 
}
