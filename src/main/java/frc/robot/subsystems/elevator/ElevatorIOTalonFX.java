package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorIOTalonFX extends SubsystemBase implements ElevatorIO{
    
    

    private final TalonFX m_left;
    private final TalonFX m_right;
    private double m_rotations;
    final MotionMagicVoltage m_request;
    //final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
    //private final PositionVoltage m_request;

    public static final double m_gearRatio = 0.5 * (1d / (1.75100 * Math.PI)) * ( 2d / 3d ) * 12;

    public ElevatorIOTalonFX() {
        
        

        m_left = new TalonFX(21, "CANivore2");
        m_right = new TalonFX(22, "CANivore2");
        
        m_request = new MotionMagicVoltage(0);//new PositionVoltage(0).withSlot(0);

        final TalonFXConfiguration elevatorMotorConfig = new TalonFXConfiguration();
        final MotorOutputConfigs rightOutputConfigs = new MotorOutputConfigs();
        final MotorOutputConfigs leftOutputConfigs = new MotorOutputConfigs();
        rightOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
        rightOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        leftOutputConfigs.NeutralMode = NeutralModeValue.Brake;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // in init function, set slot 0 gains
      
        elevatorMotorConfig.Slot0.kP = 20; 
        elevatorMotorConfig.Slot0.kI = 0; // no output for integrated error
        elevatorMotorConfig.Slot0.kD = 0.1; // A velocity of 1 rps results in 0.1 V output
        elevatorMotorConfig.Slot0.kG = 0.5;
        elevatorMotorConfig.Slot0.kS = .3;

        // TODO: Tune these values (I made them up)
        var motionMagicConfigs = elevatorMotorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 120; // Target acceleration of 160 rps/s (0.5 seconds)
        // motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
     
        m_right.setPosition(29 * m_gearRatio);
        m_left.getConfigurator().apply(elevatorMotorConfig);
        m_right.getConfigurator().apply(elevatorMotorConfig);
        m_right.getConfigurator().apply(rightOutputConfigs);
        m_left.getConfigurator().apply(leftOutputConfigs);

        m_left.setControl(new Follower(22, true));
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

    public void updateInputs(ElevatorIOInputsAutoLogged m_inputs) {
        double motorRPS = m_right.getVelocity().getValueAsDouble();
        m_inputs.elevatorRPM = motorRPS*60;
    }

 
}
