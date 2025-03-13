package frc.robot.subsystems.algae;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.algae.AlgaeIO.AlgaeIOInputs;

public class Algae extends SubsystemBase {
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable AlgaeTable = inst.getTable("Algae");
  private final DoublePublisher motorValPub = AlgaeTable.getDoubleTopic("Motor Val").publish();
  private final DoublePublisher motorVolPub = AlgaeTable.getDoubleTopic("Motor Vol").publish();
  private final DoublePublisher rotPub = AlgaeTable.getDoubleTopic("Rotations").publish();
  private final DoublePublisher setPub = AlgaeTable.getDoubleTopic("Setpoint").publish();
  private final DoublePublisher currentPub = AlgaeTable.getDoubleTopic("Current").publish();
  final double kTriggerTime = 1;

  private final AlgaeIO m_io;
  private final AlgaeIOInputsAutoLogged m_inputs = new AlgaeIOInputsAutoLogged();
  private final double DownPosition = 0;
  private final double ExtendedPosition = 4.9;// 32.5
  private Timer m_StallTimer;
  // inch to rotations of the motor
  final double m_gearRatio = 1d;

  public Algae(AlgaeIO io) {
    m_io = io;
    m_StallTimer = new Timer();
    
    // setDefaultCommand(stop());
  }

  public Command intake() {
    return this.runOnce(() -> m_io.setShootVoltage(20)).withName("Intake");
  }

  public Command stopShooter() {
    return this.runOnce(() -> m_io.setShootVoltage(0)).withName("Stop");
  }

  public Command shoot() {
    return this.runOnce(() -> m_io.setShootVoltage(-10)).withName("Shoot");
  }

  public Command moveUp() {
    return this.runOnce(() -> m_io.setPivotVoltage(1)).withName("Move Up");
  }

  public Command stopPivot() {
    return this.runOnce(() -> m_io.setPivotVoltage(0)).withName("Stop");
  }

  public Command moveDown() {
    return this.runOnce(() -> m_io.setPivotVoltage(-1)).withName("Move Down");
  }

  public Command down() {
    // return this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
    return this.runOnce(() -> {
      m_io.gotosetpoint(DownPosition, m_gearRatio);
    }).withName("Down");
  }

  public Command extend() {
    // return this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
    return this.runOnce(() -> {
      m_io.gotosetpoint(ExtendedPosition, m_gearRatio);
    }).withName("Extend");
  }

  public Command reZero() {
    return this.runOnce(() -> {
      m_io.setPosition(DownPosition * m_gearRatio);
    }).withName("ReZero");
  }

  @Override
  public void periodic() {
    // super.periodic();
    // m_io.updateInputs(m_inputs);

    motorValPub.set(m_inputs.shootVelocity);
    motorVolPub.set(m_inputs.shooterVoltage);
    setPub.set(m_inputs.wristSetpoint);
    rotPub.set(m_inputs.wristPosition);
    currentPub.set(m_inputs.shooterCurrent);

    if (m_inputs.shooterCurrent > 40) {
      m_StallTimer.start();
    } else {
      m_StallTimer.reset();
    }

    if (m_StallTimer.get() > kTriggerTime) {
      //  m_io.holdShooterPos();
    }
  }
}
