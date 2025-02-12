package frc.robot.subsystems.elevator;

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

public class ElevatorIOTalonFX extends SubsystemBase implements ElevatorIO{
    
    

    private final TalonFX m_left;
    private final TalonFX m_right;
    private double m_rotations;
    final PositionVoltage m_request;
    //final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    //private final PositionVoltage m_request;

    public ElevatorIOTalonFX() {
        
        

        m_left = new TalonFX(21, "CANivore2");
        m_right = new TalonFX(22, "CANivore2");
        
        m_request = new PositionVoltage(0).withSlot(0);

        final TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
        final MotorOutputConfigs outputConfigs = new MotorOutputConfigs();
        outputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        outputConfigs.NeutralMode = NeutralModeValue.Brake;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // in init function, set slot 0 gains
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.8; // An error of 1 rotation results in 2.4 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
        slot0Configs.kG = 0.5;
     
        m_right.setPosition(29 * (0.5 * (1d / (1.75100 * Math.PI)) * ( 2d / 3d ) * 25));
        m_left.getConfigurator().apply(elevatorMotorConfig);
        m_right.getConfigurator().apply(elevatorMotorConfig);
        m_right.getConfigurator().apply(outputConfigs);
        m_left.getConfigurator().apply(slot0Configs);
        m_right.getConfigurator().apply(slot0Configs);
        m_left.setControl(new Follower(22, true));
    }

    public void setVoltage(double voltage) {
        // return this.runOnce(()->{
            final VoltageOut request = new VoltageOut(0);
            m_left.setControl(request.withOutput(-voltage));
            m_right.setControl(request.withOutput(voltage));
        // });
        System.out.println("volts:" + m_right.getMotorVoltage());
    }
    
    public void setBrakeMode(boolean enableBrakeMode) {
        final NeutralModeValue neutralModeValue =
            enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        m_left.setNeutralMode(neutralModeValue);
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
