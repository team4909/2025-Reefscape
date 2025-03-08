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
    
    

    private final TalonFX m_front;
    private final TalonFX m_back;
    private double m_rotations;
    final PositionVoltage m_request;
    //final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    //private final PositionVoltage m_request;

    public static final double m_gearRatio = 0.5 * (1d / (1.75100 * Math.PI)) * ( 2d / 3d ) * 12;

    public ElevatorIOTalonFX() {
        
        

        m_front = new TalonFX(21, "CANivore2");
        m_back = new TalonFX(22, "CANivore2");
        
        m_request = new PositionVoltage(0).withSlot(0);

        final TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
        final MotorOutputConfigs rightOutputConfigs = new MotorOutputConfigs();
        final MotorOutputConfigs leftOutputConfigs = new MotorOutputConfigs();

        rightOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        leftOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // in init function, set slot 0 gains

        elevatorMotorConfig.Slot0.kP = 1; //3
        elevatorMotorConfig.Slot0.kI = 0; // no output for integrated error
        elevatorMotorConfig.Slot0.kD = 0.1; 
        elevatorMotorConfig.Slot0.kG = 0; //.5
        elevatorMotorConfig.Slot0.kS = 0; //.2
     
        m_back.setPosition(29 * m_gearRatio);
        m_front.getConfigurator().apply(elevatorMotorConfig);
        m_back.getConfigurator().apply(elevatorMotorConfig);

        m_back.getConfigurator().apply(rightOutputConfigs);
        m_front.getConfigurator().apply(leftOutputConfigs);

        m_front.setControl(new Follower(m_back.getDeviceID(), true));
    }

    public void setVoltage(double voltage) {
        // return this.runOnce(()->{
            final VoltageOut request = new VoltageOut(0);

            m_back.setControl(request.withOutput(voltage));
        // });
        System.out.println("volts:" + m_back.getMotorVoltage());
    }
    
    public void setBrakeMode(boolean enableBrakeMode) {
        final NeutralModeValue neutralModeValue =
            enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        m_front.setNeutralMode(neutralModeValue);
        m_back.setNeutralMode(neutralModeValue);
    }
    @Override
    public void gotosetpoint(double setpoint, double gearRatio) {
        double rotations = setpoint * gearRatio;
        m_rotations = rotations;
        System.out.println("rotations:" + rotations);
        m_back.setControl(m_request.withPosition(rotations));
    }

    public double getVelocity(){
        return m_back.getVelocity().getValueAsDouble();
    }
    public double getVoltage(){
        return m_back.getMotorVoltage().getValueAsDouble();
    }
    @Override
    public double getPosition() {
        return m_back.getPosition().getValueAsDouble();
    }

    public double getSetpoint(){
        return m_rotations;
    }

    public void setPosition(double position){
        m_back.setPosition(position);
    }

    public void updateInputs(ElevatorIOInputsAutoLogged m_inputs) {
        double motorRPS = m_back.getVelocity().getValueAsDouble();
        m_inputs.elevatorRPM = motorRPS*60;
        m_inputs.elevatorHeightInch = m_back.getPosition().getValueAsDouble() / m_gearRatio;
    }

 
}
