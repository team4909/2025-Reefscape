// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIOTalonFX;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIOPhotonVision;
import frc.robot.subsystems.Vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.AlgaeIOTalonFX;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveToPose;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;

public class RobotContainer {

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(TunerConstants.MaxSpeed * 0.1)
			.withRotationalDeadband(
					TunerConstants.MaxAngularRate * 0.1) // Add a 10% deadband
			.withDriveRequestType(
					DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

	private final DrivetrainTelemetry drivetrainTelemetry = new DrivetrainTelemetry();
	private final CommandXboxController driveController = new CommandXboxController(0);
	private final CommandXboxController zeroController = new CommandXboxController(1);

	private final SendableChooser<Command> m_chooser;

	public final CommandSwerveDrivetrain s_drivetrain = TunerConstants.createDrivetrain();
	private final Shooter s_Shooter;
	private final Elevator s_Elevator;
	private final Climber s_Climber;
	private final Vision m_vision;
	private final Algae s_Algae;

	public RobotContainer() {
		s_Shooter = new Shooter(new ShooterIOTalonFX());
		s_Elevator = new Elevator(new ElevatorIOTalonFX());
		s_Algae = new Algae(new AlgaeIOTalonFX());
		s_Climber = new Climber(new ClimberIOTalonFX());

		// Auto Named Commands
		NamedCommands.registerCommand("score", s_Shooter.shoot().withTimeout(.5));
		NamedCommands.registerCommand("feed", s_Shooter.shoot().withTimeout(1.5));
		NamedCommands.registerCommand("stop shooter", s_Shooter.stopInstant());
		NamedCommands.registerCommand("L4", s_Elevator.goToL4());
		NamedCommands.registerCommand("L3", s_Elevator.goToL3());
		NamedCommands.registerCommand("L2", s_Elevator.goToL2());
		NamedCommands.registerCommand("L1", s_Elevator.goToL1());
		NamedCommands.registerCommand("L4 Wait", s_Elevator.L4_Wait());
		NamedCommands.registerCommand("L2 Wait", s_Elevator.L2_Wait());
		NamedCommands.registerCommand("L3 Wait", s_Elevator.L3_Wait());
		NamedCommands.registerCommand("L3 Algae",
				Commands.sequence(s_Elevator.goToL3A(), s_Algae.extend(), s_Algae.intake()));
		NamedCommands.registerCommand(
				"L2 Algae", Commands
						.sequence(s_Elevator.goToL2A_wait(),
								Commands.parallel(s_Elevator.goToL2A().repeatedly(),
										Commands.sequence(s_Algae.extend(),
												s_Algae.intake())))
						.withTimeout(1));
		NamedCommands.registerCommand("Algae Stow",
				Commands.sequence(s_Algae.home(), s_Elevator.goToL1()).withTimeout(1));
		NamedCommands.registerCommand("Algae Shoot", s_Algae.shoot().withTimeout(1));
		NamedCommands.registerCommand("Algae Stop", s_Algae.stopShooter().withTimeout(0.1));
		NamedCommands.registerCommand("autoalign left", (Commands.parallel(
				new DriveToPose(s_drivetrain,
						new Transform2d(Units.inchesToMeters(-33.5 / 2 + 0.25),
								Units.inchesToMeters(13.5 + 2.25), new Rotation2d()),
						driveController, 6))));

		NamedCommands.registerCommand("autoalign right", Commands.parallel(
				new DriveToPose(s_drivetrain,
						new Transform2d(Units.inchesToMeters(-33.5 / 2 + 0.75),
								Units.inchesToMeters(0.5 + 2.25), new Rotation2d()),
						driveController, 6)));

		m_chooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", m_chooser);

		configureBindings();

		if (HALUtil.getSerialNumber().equals(TunerConstants.RobotV3)) {
			m_vision = new Vision(s_drivetrain::addVisionMeasurement,
					new VisionIOPhotonVision("front-right-cam", new Transform3d(new Translation3d(
							Units.inchesToMeters(7.16),
							Units.inchesToMeters(-10.92),
							Units.inchesToMeters(9.39)),
							new Rotation3d(
									Units.degreesToRadians(0.0),
									Units.degreesToRadians(-21.173),
									Units.degreesToRadians(-20)))),

					new VisionIOPhotonVision("front-left-cam", new Transform3d(new Translation3d(
							Units.inchesToMeters(7.211),
							Units.inchesToMeters(10.607),
							Units.inchesToMeters(9.411)),
							new Rotation3d(
									Units.degreesToRadians(0.0),
									Units.degreesToRadians(-25.414),
									Units.degreesToRadians(-50 + 4)))));

		} else if (Utils.isSimulation() || HALUtil.getSerialNumber().equals("")) {
			m_vision = new Vision(s_drivetrain::addVisionMeasurement,
					new VisionIOPhotonVisionSim("front-right-cam", new Transform3d(new Translation3d(
							Units.inchesToMeters(7.16),
							Units.inchesToMeters(-10.92),
							Units.inchesToMeters(9.39)),
							new Rotation3d(
									Units.degreesToRadians(0.0),
									Units.degreesToRadians(-21.173),
									Units.degreesToRadians(-20))),
							() -> s_drivetrain
									.getState().Pose),

					new VisionIOPhotonVisionSim("front-left-cam", new Transform3d(new Translation3d(
							Units.inchesToMeters(7.211),
							Units.inchesToMeters(10.607),
							Units.inchesToMeters(9.411)),
							new Rotation3d(
									Units.degreesToRadians(0.0),
									Units.degreesToRadians(-25.414),
									Units.degreesToRadians(-50 + 4))),
							() -> s_drivetrain.getState().Pose));

		} else if (HALUtil.getSerialNumber().equals(TunerConstants.RobotV2)) {
			m_vision = new Vision(s_drivetrain::addVisionMeasurement,
					new VisionIOPhotonVision("front-right-cam", new Transform3d(new Translation3d(
							Units.inchesToMeters(7.16),
							Units.inchesToMeters(-10.92),
							Units.inchesToMeters(9.39)),
							new Rotation3d(
									Units.degreesToRadians(0.0),
									Units.degreesToRadians(-21.173),
									Units.degreesToRadians(-20)))),

					new VisionIOPhotonVision("front-left-cam", new Transform3d(new Translation3d(
							Units.inchesToMeters(7.211),
							Units.inchesToMeters(10.607),
							Units.inchesToMeters(9.286)),
							new Rotation3d(
									Units.degreesToRadians(0.0),
									Units.degreesToRadians(-25.414),
									Units.degreesToRadians(-50)))));
		} else {
			System.out.println("Unknown Robot: '" + HALUtil.getSerialNumber() + "'");
			throw new RuntimeException("Unknown Robot Serial Number");
		}
	}

	public void stopDrive() {
		SwerveRequest.ApplyFieldSpeeds m_drive = new SwerveRequest.ApplyFieldSpeeds();
		s_drivetrain.setControl(m_drive.withSpeeds(new ChassisSpeeds(0, 0, 0)));
	}

	private Command driveWithJoystick() {
		return s_drivetrain.applyRequest(
				() -> drive
						.withVelocityX(
								-driveController.getLeftY() * TunerConstants.MaxSpeed) // Drive forward with
						// negative Y (forward)
						.withVelocityY(
								-driveController.getLeftX() * TunerConstants.MaxSpeed) // Drive left with
						// negative X (left)
						.withRotationalRate(
								-driveController.getRightX()
										* TunerConstants.MaxAngularRate) // Drive
		// counterclockwise
		// with negative X
		// (left)
		);
	}

	private void configureBindings() {
		// Note that X is defined as forward according to WPILib convention,
		// and Y is defined as to the left according to WPILib convention.
		s_drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				s_drivetrain.applyRequest(
						() -> drive
								.withVelocityX(
										-driveController.getLeftY() * TunerConstants.MaxSpeed) // Drive
								// forward
								// with
								// negative
								// Y
								// (forward)
								.withVelocityY(
										-driveController.getLeftX() * TunerConstants.MaxSpeed) // Drive
								// left
								// with
								// negative
								// X
								// (left)
								.withRotationalRate(
										-driveController.getRightX()
												* TunerConstants.MaxAngularRate) // Drive
				// counterclockwise
				// with
				// negative
				// X
				// (left)
				));

		driveController.rightStick().whileTrue(
				s_drivetrain.applyRequest(() -> drive
						.withVelocityX(
								-driveController.getLeftX() * TunerConstants.SlowSpeed)
						.withVelocityY(
								-driveController.getLeftY() * TunerConstants.SlowSpeed)
						.withRotationalRate(
								-driveController.getRightX() * TunerConstants.SlowAngularRate)));

		driveController.rightTrigger().whileTrue(new ConditionalCommand(
				s_Shooter.shoot(),
				s_Shooter.slowShoot(),
				() -> s_Elevator.isAtL4()))
				.onFalse(Commands.parallel(Commands.sequence(new WaitCommand(.5), driveWithJoystick()),
						s_Shooter.stop()));

		driveController.a().whileTrue(s_Climber.lower()).onFalse(s_Climber.stop());
		driveController.leftBumper().whileTrue(s_Elevator.goToL2().repeatedly());
		driveController.rightBumper().whileTrue(s_Elevator.goToL3().repeatedly());
		driveController.leftTrigger().whileTrue(s_Elevator.goToL4().repeatedly());
		driveController.povUp().onTrue(s_Climber.climbPosition());
		driveController.povDown().whileTrue(Commands.sequence(s_Elevator.GroundIntake_Wait(),
				Commands.parallel(s_Elevator.GroundIntake().repeatedly(),
						Commands.sequence(s_Algae.groundIntake(), s_Algae.intake()))))
				.onFalse(Commands.sequence(
						Commands.parallel(s_Algae.groundIntakeHome(), s_Elevator.L2_Wait()),
						s_Elevator.goToL1(), new WaitCommand(1), s_Algae.reZero()));
		zeroController.x().onTrue(s_Elevator.reZero());

		driveController.povLeft()
				.whileTrue(Commands.sequence(s_Elevator.goToL2A_wait(),
						Commands.parallel(s_Elevator.goToL2A().repeatedly(),
								Commands.sequence(s_Algae.extend(), s_Algae.intake()))))
				.onFalse((s_Algae.home()));
		driveController.povRight()
				.whileTrue(Commands.sequence(s_Elevator.goToL3A_wait(),
						Commands.parallel(s_Elevator.goToL3A().repeatedly(),
								Commands.sequence(s_Algae.extend(), s_Algae.intake()))))
				.onFalse((s_Algae.home()));
		driveController.rightStick().onTrue(s_Algae.shoot()).onFalse(s_Algae.stopShooter());

		driveController.leftStick()
				.whileTrue(s_Algae.shootL1low().andThen(
						s_Algae.slowShoot().andThen(s_Elevator.goToL1Shoot().repeatedly())))
				.onFalse(s_Algae.home().andThen(s_Algae.stopShooter()
						.andThen(new WaitCommand(5).alongWith(s_Elevator.goToL1()))));

		zeroController.a().onTrue(s_Climber.reZero());
		zeroController.b().onTrue(s_Algae.reZero());

		// auto align right
		// positive moves right for second param of translation
		driveController.b().whileTrue(Commands.parallel(
				new DriveToPose(s_drivetrain, new Transform2d(Units.inchesToMeters(-33.5 / 2 - 1.5),
						Units.inchesToMeters(0.5 + 2.25 + .25), Rotation2d.fromDegrees(-5)),
						driveController, 8),
				s_Shooter.shoot()))
				.onFalse(new InstantCommand(() -> driveController.setRumble(RumbleType.kBothRumble, 0))
						.andThen(Commands.parallel(s_Shooter.stop(),
								new RunCommand(() -> stopDrive(), s_drivetrain))));

		// stop the robot override
		driveController.button(7).whileTrue(new RunCommand(() -> {

			stopDrive();
		}, s_drivetrain));

		// auto align left
		driveController.x().whileTrue(Commands.parallel(new DriveToPose(s_drivetrain,
				new Transform2d(Units.inchesToMeters(-33.5 / 2 + 0.75),
						Units.inchesToMeters(13.5 + 2.25),
						new Rotation2d()),
				driveController, 6), s_Shooter.shoot()))
				.onFalse(new InstantCommand(() -> driveController.setRumble(RumbleType.kBothRumble, 0))
						.andThen(Commands.parallel(s_Shooter.stop(),
								new RunCommand(() -> stopDrive(), s_drivetrain))));

		driveController.y().whileTrue(new AutoClimbCommand(driveController, s_drivetrain));

		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		// joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		// joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		// joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		// joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		// reset the field-centric heading on left bumper press
		driveController.start().onTrue(s_drivetrain.runOnce(() -> s_drivetrain.seedFieldCentric()));

		s_drivetrain.registerTelemetry(drivetrainTelemetry::telemeterize);
	}

	public Command getAutonomousCommand() {
		return m_chooser.getSelected();
	}

}
