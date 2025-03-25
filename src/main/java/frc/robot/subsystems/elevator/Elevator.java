package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable elevatorTable = inst.getTable("Elevator");

  private ElevatorIOInputsAutoLogged m_inputs = new ElevatorIOInputsAutoLogged();
  private final ElevatorIO m_io;
  // GSD setpoints
  private final double L1Setpoint = 29.48;
  private final double L2Setpoint = 36   ;
  private final double L3Setpoint = 52;
  private final double L4Setpoint = 77;

  private final double L2ASetpoint = 48.5+3+1;
  private final double L3ASetpoint = 66.5+1.5;

  CANBus canBus = new CANBus("CANivore2");

  // private final double L1Setpoint = 29;
  // private final double L2Setpoint = 33;
  // private final double L3Setpoint = 40;
  // private final double L4Setpoint = 50;
  // private final double L2ASetpoint = 2.5;
  // private final double L3ASetpoint = 3.5;
  // inch to rotations of the motor

  public Elevator(ElevatorIO io) {
    super("Elevator");
    m_io = io;
    setDefaultCommand(this.run(()-> m_io.gotosetpoint(L1Setpoint, ElevatorIOTalonFX.m_gearRatio)).withName("Elevator Default Command"));
  }

  // public Command moveUp() {
  // return this.runOnce(() -> m_io.setVoltage(3)).withName("UP");
  // }

  public Command stop() {
    return this.runOnce(() -> m_io.setVoltage(0)).withName("Stop");
  }

  public Command moveDown() {
    return this.runOnce(() -> m_io.setVoltage(-3)).withName("Move Down");
  }

  public Command goToL1() {
    // return this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
    return this.runOnce(() -> {
      m_io.gotosetpoint(L1Setpoint, ElevatorIOTalonFX.m_gearRatio);
    }).withName("L1");
  }

  public Command goToL2() {
    // return this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
    return this.runOnce(() -> {
      m_io.gotosetpoint(L2Setpoint, ElevatorIOTalonFX.m_gearRatio);
    }).withName("L2");
  }

  public Command goToL3() {
    // return this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
    return this.runOnce(() -> {
      m_io.gotosetpoint(L3Setpoint, ElevatorIOTalonFX.m_gearRatio);
    }).withName("L3");
  }

  public Command goToL4() {
    // return this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
    return this.runOnce(() -> {
      m_io.gotosetpoint(L4Setpoint, ElevatorIOTalonFX.m_gearRatio);
    }).withName("L4");
  }

  public Command L4_Wait() {
    // return this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
    return this.run(() -> {
      SmartDashboard.putString("L4Wait", "Start");
      m_io.gotosetpointWithSlot(L4Setpoint, ElevatorIOTalonFX.m_gearRatio,2);
    }).withName("L4").until(() -> {
      // SmartDashboard.putNumber("Elevator/l4wait", Math.abs(L4Setpoint - m_inputs.elevatorHeightInch) );
      // SmartDashboard.putNumber("Elevator/actual", m_inputs.elevatorHeightInch);
      // SmartDashboard.putNumber("Elevator/target", L4Setpoint);
      return Math.abs(L4Setpoint - m_inputs.heightInch) < 0.1;
    }).andThen(()-> SmartDashboard.putString("L4Wait", "End"));
  }

  public Command L2_Wait() {
    // return this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
    return this.run(() -> {
      SmartDashboard.putString("L2Wait", "Start");
      m_io.gotosetpoint(L2Setpoint, ElevatorIOTalonFX.m_gearRatio);
    }).withName("L2Wait").until(() -> {
      // SmartDashboard.putNumber("Elevator/l4wait", Math.abs(L4Setpoint - m_inputs.elevatorHeightInch) );
      // SmartDashboard.putNumber("Elevator/actual", m_inputs.elevatorHeightInch);
      // SmartDashboard.putNumber("Elevator/target", L4Setpoint);
      return Math.abs(L2Setpoint - m_inputs.heightInch) < 0.1;
    }).andThen(()-> SmartDashboard.putString("L2Wait", "End"));
  }

  public Command L3_Wait() {
    // return this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
    return this.run(() -> {
      SmartDashboard.putString("L3Wait", "Start");
      m_io.gotosetpoint(L3Setpoint, ElevatorIOTalonFX.m_gearRatio);
    }).withName("L3Wait").until(() -> {
      return Math.abs(L3Setpoint - m_inputs.heightInch) < 0.1;
    }).andThen(()-> SmartDashboard.putString("L3Wait", "End"));
  }


  public Command goToL3A() {
    // return this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
    return this.runOnce(() -> {
      m_io.gotosetpoint(L3ASetpoint, ElevatorIOTalonFX.m_gearRatio);
    }).withName("L3A");
  }

  public Command goToL2A() {
    // return this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
    return this.runOnce(() -> {
      m_io.gotosetpoint(L2ASetpoint, ElevatorIOTalonFX.m_gearRatio);
    }).withName("L2A");
  }

  public Command goToL2A_wait() {
    // return this.run(() -> m_io.gotosetpoint(L1Setpoint,m_gearRatio));
    return this.run(() -> {
      m_io.gotosetpoint(L2Setpoint, ElevatorIOTalonFX.m_gearRatio);
    }).withName("L2AWait").until(() -> {
      return Math.abs(L2ASetpoint - m_inputs.heightInch) < 0.1;
    });
  }

  public Command goUpInch() {
    return this.runOnce(() -> {
      m_io.gotosetpoint(m_io.getPosition() + (1 / 10), ElevatorIOTalonFX.m_gearRatio);
    }).withName("Inch");
  }

  public Command reZero() {
    return this.runOnce(() -> {
      m_io.setPosition(L1Setpoint * ElevatorIOTalonFX.m_gearRatio);
    }).withName("ReZero");
  }

  public Command testPrint() {
    return this.runOnce(() -> {
      System.out.println("Test");
    }).withName("Test");
  }

  @Override
  public void periodic() {
    super.periodic();
    m_io.updateInputs(m_inputs);
    Logger.processInputs(getName(), m_inputs);
    var status = canBus.getStatus();
    SmartDashboard.putString("Canivore2/status", status.Status.toString());
    SmartDashboard.putBoolean("Canivore2/isNetworkFD", canBus.isNetworkFD());
    SmartDashboard.putNumber("Canivore2/BusUtilization", status.BusUtilization);
    SmartDashboard.putNumber("Canivore2/BusOffCount", status.BusOffCount);
    SmartDashboard.putNumber("Canivore2/TxFullCount", status.TxFullCount);

    // motorValPub.set(m_io.getVelocity());
    // motorVolPub.set(m_io.getVoltage());
    // setPub.set(m_io.getSetpoint());
    // rotPub.set(m_io.getPosition());

    // if (this.getCurrentCommand() != null) {
    //   SmartDashboard.putString("elevator/command", this.getCurrentCommand().getName());
    // } else {
    //   SmartDashboard.putString("elevator/command", "null");
    // }
  }

  public boolean isAtL1() {
    return Math.abs(L1Setpoint - m_inputs.heightInch) < 0.1;
  }

  public boolean isAtL4() {
    return Math.abs(L4Setpoint - m_inputs.heightInch) < 0.1;
  }
}
