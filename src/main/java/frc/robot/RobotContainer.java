// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drivetrain.DriveToPose;
import frc.robot.subsystems.drivetrain.ReefBranchAlign;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionIOPhotonVision;
import frc.robot.subsystems.algae.Algae;
import frc.robot.subsystems.algae.AlgaeIOTalonFX;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIOTalonFX;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveToFieldPose;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double SlowSpeed = TunerConstants.kSlowSpeed.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75)
            .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private double SlowAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(
                    DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController zeroController = new CommandXboxController(1);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> m_chooser;
    private final Shooter s_Shooter;
    private final Elevator s_Elevator;
  private final Climber s_Climber;
    private final Vision m_vision;
    private final Algae s_Algae;
    public static AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);


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
        NamedCommands.registerCommand("test", s_Elevator.testPrint());
        NamedCommands.registerCommand("L4 Wait", s_Elevator.L4_Wait());
        NamedCommands.registerCommand("L2 Wait", s_Elevator.L2_Wait());
        NamedCommands.registerCommand("L3 Wait", s_Elevator.L3_Wait());
        NamedCommands.registerCommand("L3 Algae", Commands.sequence(s_Elevator.goToL3A(), s_Algae.extend(), s_Algae.intake()));
        NamedCommands.registerCommand("L2 Algae", Commands.sequence(s_Elevator.goToL2A(), s_Algae.extend(), s_Algae.intake()).withTimeout(1));
        NamedCommands.registerCommand("Algae Stow", Commands.sequence(s_Algae.down(), s_Elevator.goToL1()).withTimeout(.1));
        NamedCommands.registerCommand("Algae Shoot", s_Algae.shoot().withTimeout(1));
        NamedCommands.registerCommand("autoalign left", new ReefBranchAlign(drivetrain,
        new Transform2d(Units.inchesToMeters(-4.5), Units.inchesToMeters(13.5+2.25), new Rotation2d()),
        () -> -joystick.getLeftY()));

        NamedCommands.registerCommand("autoalign right", new ReefBranchAlign(drivetrain,
        new Transform2d(Units.inchesToMeters(-4.5), Units.inchesToMeters(0.5+2.25), new Rotation2d()),
        () -> -joystick.getLeftY()));

        m_chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", m_chooser);


        configureBindings();

        if (HALUtil.getSerialNumber().equals(TunerConstants.RobotV3)) {
            m_vision = new Vision(drivetrain::addVisionMeasurement,

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
                                    Units.degreesToRadians(-50)))));
        } 
        else if (HALUtil.getSerialNumber().equals(TunerConstants.RobotV2)) {
            m_vision = new Vision(drivetrain::addVisionMeasurement,

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
                                    Units.degreesToRadians(-50)))));
        }
        else { 
            System.out.println("Unknown Robot");
            throw new RuntimeException("Unknown Robot Serial Number");
        }
    }

    public void periodic() {
        // System.out.println("Elevator Command"+s_Elevator.getCurrentCommand());


        // check if elevator is at L1, if so run the command that looks for current spike running intake
        // if (s_Elevator.isAtL1()) {
        //     s_Shooter.setDefaultRunToCurrentSpike();
        // }
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive
                                .withVelocityX(
                                        -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                                .withVelocityY(
                                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                                .withRotationalRate(
                                        -joystick.getRightX()
                                                * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.rightStick().whileTrue(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(
                                -joystick.getLeftX() * SlowSpeed)
                        .withVelocityY(
                                -joystick.getLeftY() * SlowSpeed)
                        .withRotationalRate(
                                -joystick.getRightX() * SlowAngularRate)));
        
        joystick.rightTrigger().whileTrue(new ConditionalCommand(
                s_Shooter.shoot(), 
                s_Shooter.slowShoot(), 
                () -> s_Elevator.isAtL4()
        )).onFalse(s_Shooter.stop());
    

        // joystick.y().whileTrue(s_Shooter.slowShoot()).onFalse(s_Shooter.stop());


        joystick.a().whileTrue(s_Climber.lower()).onFalse(s_Climber.stop());
        joystick.leftStick().onTrue(s_Climber.winchedPosition());

        joystick.b().onTrue(s_Elevator.goToL1()).onFalse(s_Elevator.stop());
        joystick.leftBumper().onTrue(s_Elevator.goToL2()).onFalse(s_Elevator.goToL1());
        joystick.rightBumper().onTrue(s_Elevator.goToL3()).onFalse(s_Elevator.goToL1());
        joystick.leftTrigger().onTrue(s_Elevator.goToL4()).onFalse(s_Elevator.goToL1());
        joystick.povUp().onTrue(s_Climber.climbPosition());
        joystick.povDown().whileTrue(s_Elevator.moveDown()).onFalse(s_Elevator.stop());
        zeroController.x().onTrue(s_Elevator.reZero());

        // joystick.x().onTrue(s_Elevator.goToL3A()).onFalse(s_Elevator.goToL1());
        //joystick.a().onTrue(s_Elevator.goToL2A()).onFalse(s_Elevator.goToL1());
        joystick.povLeft().onTrue(Commands.sequence(s_Elevator.goToL2A(), s_Algae.extend(), s_Algae.intake())).onFalse(Commands.sequence(s_Algae.down(), s_Elevator.goToL1()));
        joystick.povRight().onTrue(Commands.sequence(s_Elevator.goToL3A(), s_Algae.extend(), s_Algae.intake())).onFalse(Commands.sequence(s_Algae.down(), s_Elevator.goToL1()));
        joystick.rightStick().onTrue(s_Algae.shoot()).onFalse(s_Algae.stopShooter());
       // joystick.leftStick().onTrue(s_Algae.intake()).onFalse(s_Algae.stopShooter());
        zeroController.a().onTrue(s_Climber.reZero());
        // joystick

        // .b()
        // .whileTrue(
        // drivetrain.applyRequest(
        // () ->
        // point.withModuleDirection(
        // new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // joystick.b().whileTrue(new DriveToPose(drivetrain,
        //         new Transform2d(Units.inchesToMeters(-4.5), Units.inchesToMeters(0.5), new Rotation2d())));
        // joystick.x().whileTrue(new DriveToPose(drivetrain,
        //         new Transform2d(Units.inchesToMeters(-4.5), Units.inchesToMeters(13.5), new Rotation2d())));
        zeroController.b().onTrue(s_Algae.reZero());

        // joystick.b().whileTrue(Commands.sequence(
        //         s_Shooter.setDefaultDoNotRun(), 
        //         Commands.parallel(
        //                 new ReefBranchAlign(drivetrain, new Transform2d(Units.inchesToMeters(-4.5), Units.inchesToMeters(0.5+2.25), new Rotation2d()),() -> -joystick.getLeftY()),
        //                 s_Shooter.shootTrough()
        //                 )
        //         )).onFalse(s_Shooter.stop());

        // joystick.x().whileTrue(Commands.sequence(
        //         s_Shooter.setDefaultDoNotRun(), 
        //         Commands.parallel(      
        //                 new ReefBranchAlign(drivetrain, new Transform2d(Units.inchesToMeters(-4.5), Units.inchesToMeters(13.5+2.25), new Rotation2d()),() -> -joystick.getLeftY()),
        //                 s_Shooter.shootTrough()
        //         ))).onFalse(s_Shooter.stop());

        joystick.b().whileTrue(Commands.parallel(
                new DriveToPose(drivetrain, new Transform2d(Units.inchesToMeters(-33.5/2+0.75), Units.inchesToMeters(0.5+2.25), new Rotation2d()), joystick), 
                s_Shooter.shoot()))
                .onFalse(new InstantCommand(()->joystick.setRumble(RumbleType.kBothRumble, 0)).andThen(s_Shooter.stop()));

        joystick.x().whileTrue(Commands.parallel(new DriveToPose(drivetrain,
                new Transform2d(Units.inchesToMeters(-33.5/2+0.75), Units.inchesToMeters(13.5+2.25), new Rotation2d()), joystick),s_Shooter.shoot())).onFalse(new InstantCommand(()->joystick.setRumble(RumbleType.kBothRumble, 0)).andThen(s_Shooter.stop()));

        // joystick.y().whileTrue(new DriveToFieldPose(drivetrain,
        //         new Pose2d(7.495, 5.026, Rotation2d.fromDegrees(-90)), joystick));

        var startPose = new Pose2d(7.495, 5.026, Rotation2d.fromDegrees(-90));
        var goToClimbStartPose = new DriveToFieldPose(drivetrain,startPose, joystick,3);

        var endPose = new Pose2d(8.9, 5.026, Rotation2d.fromDegrees(-90));
        var goToClimbEndPose = new DriveToFieldPose(drivetrain,endPose, joystick,1.5);

        var ClimbMidPose = new Pose2d(8.640, 5.026, Rotation2d.fromDegrees(-90));
        var goToClimbMidPose = new DriveToFieldPose(drivetrain,ClimbMidPose, joystick,3);

        

        // joystick.y().whileTrue(new ConditionalCommand(goToClimbStartPose, goToClimbEndPose, ()-> !poseEqualsPoseWithDelta(drivetrain.getState().Pose, startPose)));                 
        joystick.y().whileTrue(new ConditionalCommand(goToClimbStartPose,
                new ConditionalCommand(goToClimbEndPose, goToClimbMidPose,()-> !poseEqualsPoseWithDelta(drivetrain.getState().Pose, endPose))
                , ()-> !poseEqualsPoseWithDelta(drivetrain.getState().Pose, startPose ) && !poseEqualsPoseWithDelta(drivetrain.getState().Pose, endPose )));                 
        
        // joystick.x().whileTrue(new DriveToPose( new Pose2d(
        // Units.inchesToMeters(144.003)-Units.inchesToMeters(13),
        // Units.inchesToMeters(158.500),
        // Rotation2d.fromDegrees(0)), drivetrain));

        // joystick.x().whileTrue(s_Shooter.shootL2()).onFalse(s_Shooter.stop());
        // joystick.y().whileTrue(s_Shooter.intake()).onFalse(s_Shooter.stop());
        // joystick.a().whileTrue(s_Shooter.shootTrough()).onFalse(s_Shooter.stop());

        // joystick.povUp().whileTrue(s_Elevator.moveUp()).onFalse(s_Elevator.stop());
        // joystick.povDown().whileTrue(s_Elevator.moveDown()).onFalse(s_Elevator.stop());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public boolean poseEqualsPoseWithDelta(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation()) < Units.inchesToMeters(2)
                && (a.getRotation().getDegrees() - b.getRotation().getDegrees() < 5);
    }

    public Command getAutonomousCommand() {
        // return new PathPlannerAuto("cut");
        return m_chooser.getSelected();
    }


}
