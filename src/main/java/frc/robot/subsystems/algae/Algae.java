package frc.robot.subsystems.algae;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {

	private final AlgaeIO m_io;
	private final AlgaeIOInputsAutoLogged m_inputs = new AlgaeIOInputsAutoLogged();

	final double kTriggerTime = 1;
	private final double DownPosition = 0;
	private final double ExtendedPosition = 9;
	private final double GroundIntake = 17;

	private Timer m_StallTimer = new Timer();
	// inch to rotations of the motor
	final double m_gearRatio = 1d;

	public Algae(AlgaeIO io) {
		super("Algea");
		m_io = io;
	}

	public Command intake() {
		return this.runOnce(() -> m_io.setShootVoltage(20)).withName("Intake");
	}

	public Command slowShoot() {
		return this.runOnce(() -> m_io.setShootVoltage(.5)).withName("Intake");
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

	public Command holdCoral() {
		return this.run(() -> m_io.setShootVoltage(-10));
	}

	public Command moveDown() {
		return this.runOnce(() -> m_io.setPivotVoltage(-1)).withName("Move Down");
	}

	public Command home() {
		return this.runOnce(() -> {
			m_io.gotosetpoint(DownPosition, m_gearRatio);
		}).withName("Down");
	}

	public Command groundIntakeHome() {
		return this.runOnce(() -> {
			m_io.gotosetpointslow(DownPosition, m_gearRatio, -0.1);
		}).withName("Down");
	}

	public Command extend() {
		return this.runOnce(() -> {
			m_io.gotosetpoint(ExtendedPosition, m_gearRatio);
		}).withName("Extend");
	}

	public Command groundIntake() {
		return this.runOnce(() -> {
			m_io.gotosetpoint(GroundIntake, m_gearRatio);
		}).withName("Ground Intake");
	}

	public Command intakeL1() {
		return this.runOnce(() -> {
			m_io.gotosetpoint(3, m_gearRatio);
		}).withName("IntakeL1");
	}

	public Command shootL1low() {
		return this.runOnce(() -> {
			m_io.gotosetpoint(7, m_gearRatio);
		}).withName("IntakeL1");
	}

	public Command reZero() {
		return this.runOnce(() -> {
			m_io.setPosition(DownPosition * m_gearRatio);
		}).withName("ReZero");
	}

	@Override
	public void periodic() {
		m_io.updateInputs(m_inputs);

		if (m_inputs.shooterCurrent > 40) {
			m_StallTimer.start();
		} else {
			m_StallTimer.reset();
		}

		if (m_StallTimer.get() > kTriggerTime) {
			// m_io.holdShooterPos();
		}
	}
}
