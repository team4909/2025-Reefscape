package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorIOTalonFX extends SubsystemBase implements ElevatorIO{
    private final TalonFX m_leftMotor;
    private final TalonFX m_rightMotor;
    
    private MotionMagicVoltage m_elevatorVoltageRequest;
    private VoltageOut m_voltageRequest;

    private final StatusSignal<Voltage> m_rightMotorVolts, m_leftMotorVolts;
    private final StatusSignal<Angle> m_position;
    private final StatusSignal<AngularVelocity> m_velocity;

    public ElevatorIOTalonFX() {
        m_leftMotor = new TalonFX(21, "CANivore2");
        m_rightMotor = new TalonFX(22, "CANivore2");
        m_elevatorVoltageRequest = new MotionMagicVoltage(0);
        m_voltageRequest = new VoltageOut(2);
        
        TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        var slot0Configs = motorConfiguration.Slot0;

        //TODO: Run sysid and get these values
        slot0Configs.kS = 0.115; //Voltage to overcome friction
        slot0Configs.kV = 1.99938560282; //Voltage to reach velocity target of 1 rps
        slot0Configs.kA = 0.00139719469; //Voltage to reach an acceleration of 1rps/s
        slot0Configs.kG = 0.125; //Voltage to overcome gravity
        slot0Configs.kP = 24.0;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.0;

        // //TODO: Tune these values (I mde them up)
        var motionMagicConfigs = motorConfiguration.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 40; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 60; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        //Other settings:
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = 40.0;
        motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        m_rightMotor.getConfigurator().apply(motorConfiguration);
        m_leftMotor.getConfigurator().apply(motorConfiguration);
     
        m_rightMotor.setPosition(29 * (0.5 * (1d / (1.75100 * Math.PI)) * ( 2d / 3d ) * 25));
        m_leftMotor.setPosition(29 * (0.5 * (1d / (1.75100 * Math.PI)) * ( 2d / 3d ) * 25));

        m_leftMotor.setControl(new Follower(m_rightMotor.getDeviceID(), true));

        //Logged Values
        m_rightMotorVolts = m_rightMotor.getMotorVoltage();
        m_leftMotorVolts = m_rightMotor.getMotorVoltage();
        m_velocity = m_rightMotor.getVelocity();
        m_position = m_rightMotor.getPosition();
    }

    @Override
    public void setVoltage(double voltage) {
        m_rightMotor.setControl(m_voltageRequest.withOutput(voltage));
    }
    
    public void setBrakeMode(boolean enableBrakeMode) {
        NeutralModeValue neutralModeValue =
            enableBrakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        m_leftMotor.setNeutralMode(neutralModeValue);
        m_rightMotor.setNeutralMode(neutralModeValue);
    }

    @Override
    public void gotosetpoint(double setpoint, double gearRatio) {
        double rotations = setpoint * gearRatio;
        m_rightMotor.setControl(m_elevatorVoltageRequest.withPosition(rotations));
    }

    public void setPosition(double position){
        m_rightMotor.setPosition(position);
        m_leftMotor.setPosition(position);
    }

    public double getPosition(){
        return m_rightMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leftMotorConnected = BaseStatusSignal.refreshAll(m_leftMotorVolts).isOK();
        inputs.rightMotorConnected = BaseStatusSignal.refreshAll(m_rightMotorVolts, m_velocity, m_position).isOK();

        inputs.goalPosition = m_elevatorVoltageRequest.getPositionMeasure().magnitude();
        inputs.rightMotorVoltage = m_rightMotorVolts.getValueAsDouble();
        inputs.leftMotorVoltage = m_leftMotorVolts.getValueAsDouble();
        inputs.velocity = m_velocity.getValueAsDouble();
        inputs.motorPosition = m_position.getValueAsDouble();
    }
 
}
