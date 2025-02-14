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
    
    

    private final TalonFX m_algaeshootmotor;
    private final TalonFX m_algaepivotmotor;
    private double m_rotations;
    final PositionVoltage m_request;
    //final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    //private final PositionVoltage m_request;

    public AlgaeIOTalonFX() {
        
        

        m_algaeshootmotor = new TalonFX(23, "CANivore2");
        m_algaepivotmotor = new TalonFX(24, "CANivore2");
        
        m_request = new PositionVoltage(0).withSlot(0);

        final TalonFXConfiguration algaeMotorConfig = new TalonFXConfiguration();
        final MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        outputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        outputConfigs.NeutralMode = NeutralModeValue.Brake;
        algaeMotorConfig.CurrentLimits.SupplyCurrentLimit = 5.0;
        algaeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.5; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output
        slot0Configs.kG = 0;
     
        m_algaepivotmotor.setPosition(0);
        m_algaeshootmotor.getConfigurator().apply(algaeMotorConfig);
        m_algaepivotmotor.getConfigurator().apply(algaeMotorConfig);
        m_algaeshootmotor.getConfigurator().apply(outputConfigs);
        m_algaepivotmotor.getConfigurator().apply(outputConfigs);
        m_algaeshootmotor.getConfigurator().apply(slot0Configs);
        m_algaepivotmotor.getConfigurator().apply(slot0Configs);
    }

    public void setShootVoltage(double voltage) {
        // return this.runOnce(()->{
            final VoltageOut request = new VoltageOut(0);
            m_algaeshootmotor.setControl(request.withOutput(voltage));
        // });
        System.out.println("volts:" + m_algaeshootmotor.getMotorVoltage());
    }

    public void setPivotVoltage(double voltage) {
        // return this.runOnce(()->{
            final VoltageOut request = new VoltageOut(0);
            m_algaepivotmotor.setControl(request.withOutput(voltage));
        // });
        System.out.println("volts:" + m_algaepivotmotor.getMotorVoltage());
    }
    
    public void setBrakeMode(boolean enableBrakeMode) {
        final NeutralModeValue neutralModeValue =
            enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        m_algaeshootmotor.setNeutralMode(neutralModeValue);
    }
    @Override
    public void gotosetpoint(double setpoint, double gearRatio) {
        double rotations = setpoint * gearRatio;
        m_rotations = rotations;
        System.out.println("rotations:" + rotations);
        m_algaepivotmotor.setControl(m_request.withPosition(rotations));

    }

    public double getVelocity(){
        return m_algaeshootmotor.getVelocity().getValueAsDouble();
    }
    public double getVoltage(){
        return m_algaeshootmotor.getMotorVoltage().getValueAsDouble();
    }
    @Override
    public double getPosition() {
        return m_algaepivotmotor.getPosition().getValueAsDouble();
    }

    public double getSetpoint(){
        return m_rotations;
    }

    public void setPosition(double position){
        m_algaeshootmotor.setPosition(position);
    }
 
}
