package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;

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

	// old worlds setpoints
	// private final double L1Setpoint = 29; //29.48;
	// private final double L2Setpoint = 37.2;
	// private final double L3Setpoint = 53.2;
	// private final double L4Setpoint = 76.75;
	// private final double DCMPL4Setpoint = 76;

	private final double L1Intake = 42;
	private final double L1Shoot = 32;

	// private final double L2ASetpoint = 48.5+3+1;
	// private final double L3ASetpoint = 66.5+1.5;

	// UVM Setpoints
	private final double L1Setpoint = 29; // 29.48;
	private final double L2Setpoint = 36;
	private final double L3Setpoint = 52;
	private final double L4Setpoint = 77.25;

	private final double L2ASetpoint = 48.5 + 3 + 1;
	private final double L3ASetpoint = 66.5 + 1.5;

	private final double GroundIntakeSetpoint = 35.2;

	CANBus canBus = new CANBus("CANivore2");

	public Elevator(ElevatorIO io) {
		super("Elevator");
		m_io = io;
		setDefaultCommand(goToL1());
	}

	public Command stop() {
		return this.runOnce(() -> m_io.setVoltage(0)).withName("Stop");
	}

	public Command moveDown() {
		return this.runOnce(() -> m_io.setVoltage(-3)).withName("Move Down");
	}

	public Command goToL1() {
		return this.runOnce(() -> {
			m_io.gotosetpoint(L1Setpoint, ElevatorIOTalonFX.m_gearRatio);
		}).withName("L1");
	}

	public Command goToL2() {
		return this.runOnce(() -> {
			m_io.gotosetpoint(L2Setpoint, ElevatorIOTalonFX.m_gearRatio);
		}).withName("L2");
	}

	public Command goToL3() {
		return this.runOnce(() -> {
			m_io.gotosetpoint(L3Setpoint, ElevatorIOTalonFX.m_gearRatio);
		}).withName("L3");
	}

	public Command goToL4() {
		return this.runOnce(() -> {
			m_io.gotosetpoint(L4Setpoint, ElevatorIOTalonFX.m_gearRatio);
		}).withName("L4");
	}

	public Command goToL1Intake() {
		return this.runOnce(() -> {
			m_io.gotosetpoint(L1Intake, ElevatorIOTalonFX.m_gearRatio);
		}).withName("L1Intake");
	}

	public Command goToL1Shoot() {
		return this.runOnce(() -> {
			m_io.gotosetpoint(L1Shoot, ElevatorIOTalonFX.m_gearRatio);
		}).withName("L1Shoot");
	}

	public Command L4_Wait() {
		return this.run(() -> {
			SmartDashboard.putString("L4Wait", "Start");
			m_io.gotosetpointWithSlot(L4Setpoint, ElevatorIOTalonFX.m_gearRatio, 2);
		}).withName("L4").until(() -> {
			return Math.abs(L4Setpoint - m_inputs.heightInch) <= 0.1;
		}).andThen(() -> SmartDashboard.putString("L4Wait", "End"));
	}

	public Command L2_Wait() {
		return this.run(() -> {
			SmartDashboard.putString("L2Wait", "Start");
			m_io.gotosetpoint(L2Setpoint, ElevatorIOTalonFX.m_gearRatio);
		}).withName("L2Wait").until(() -> {
			return Math.abs(L2Setpoint - m_inputs.heightInch) < 0.1;
		}).andThen(() -> SmartDashboard.putString("L2Wait", "End"));
	}

	public Command L3_Wait() {
		return this.run(() -> {
			SmartDashboard.putString("L3Wait", "Start");
			m_io.gotosetpoint(L3Setpoint, ElevatorIOTalonFX.m_gearRatio);
		}).withName("L3Wait").until(() -> {
			return Math.abs(L3Setpoint - m_inputs.heightInch) < 0.1;
		}).andThen(() -> SmartDashboard.putString("L3Wait", "End"));
	}

	public Command GroundIntake() {
		return this.runOnce(() -> {
			m_io.gotosetpoint(GroundIntakeSetpoint, ElevatorIOTalonFX.m_gearRatio);
		}).withName("GroundIntake");
	}

	public Command GroundIntake_Wait() {
		return this.run(() -> {
			SmartDashboard.putString("GroundIntake", "Start");
			m_io.gotosetpoint(GroundIntakeSetpoint, ElevatorIOTalonFX.m_gearRatio);
		}).withName("GroundIntake").until(() -> {
			return Math.abs(GroundIntakeSetpoint - m_inputs.heightInch) < 0.1;
		}).andThen(() -> SmartDashboard.putString("GroundIntake", "End"));
	}

	public Command goToL3A() {
		return this.runOnce(() -> {
			m_io.gotosetpoint(L3ASetpoint, ElevatorIOTalonFX.m_gearRatio);
		}).withName("L3A");
	}

	public Command goToL2A() {
		return this.runOnce(() -> {
			m_io.gotosetpoint(L2ASetpoint, ElevatorIOTalonFX.m_gearRatio);
		}).withName("L2A");
	}

	public Command goToL2A_wait() {
		return this.run(() -> {
			m_io.gotosetpoint(L2ASetpoint, ElevatorIOTalonFX.m_gearRatio);
		}).withName("L2AWait").until(() -> {
			return Math.abs(L2ASetpoint - m_inputs.heightInch) < 0.1;
		});
	}

	public Command goToL3A_wait() {
		return this.run(() -> {
			m_io.gotosetpoint(L3ASetpoint, ElevatorIOTalonFX.m_gearRatio);
		}).withName("L3AWait").until(() -> {
			return Math.abs(L3ASetpoint - m_inputs.heightInch) < 0.1;
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
		Logger.recordOutput("Canivore2/status", status.Status.toString());
		Logger.recordOutput("Canivore2/isNetworkFD", canBus.isNetworkFD());
		Logger.recordOutput("Canivore2/BusUtilization", status.BusUtilization);
		Logger.recordOutput("Canivore2/BusOffCount", status.BusOffCount);
		Logger.recordOutput("Canivore2/TxFullCount", status.TxFullCount);

	}

	public boolean isAtL1() {
		return Math.abs(L1Setpoint - m_inputs.heightInch) < 0.1;
	}

	public boolean isAtL4() {
		return Math.abs(L4Setpoint - m_inputs.heightInch) < 0.1;
	}
}
