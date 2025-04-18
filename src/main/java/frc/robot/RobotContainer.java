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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.drivetrain.DriveToPose;
import frc.robot.subsystems.drivetrain.ReefBranchAlign;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionConstants;
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
        NamedCommands.registerCommand("L2 Algae", Commands.sequence(s_Elevator.goToL2A_wait(), Commands.parallel(s_Elevator.goToL2A().repeatedly(), Commands.sequence(s_Algae.extend(), s_Algae.intake()))).withTimeout(1));
        NamedCommands.registerCommand("Algae Stow", Commands.sequence(s_Algae.home(), s_Elevator.goToL1()).withTimeout(1));
       // NamedCommands.registerCommand("Algae Hold", Commands.sequence(s_Algae.home(),s_Elevator.goToL2A_wait).withTimeout(.5));
        NamedCommands.registerCommand("Algae Shoot", s_Algae.shoot().withTimeout(1));
        NamedCommands.registerCommand("Algae Stop", s_Algae.stopShooter().withTimeout(0.1));
        NamedCommands.registerCommand("autoalign left", (Commands.parallel(
                new DriveToPose(drivetrain, new Transform2d(Units.inchesToMeters(-33.5/2+0.25), Units.inchesToMeters(0.5+2.25-.5), new Rotation2d()), joystick, 6))));

        NamedCommands.registerCommand("autoalign right", Commands.parallel(
        new DriveToPose(drivetrain, new Transform2d(Units.inchesToMeters(-33.5/2+0.75), Units.inchesToMeters(0.5+2.25), new Rotation2d()), joystick, 6)));

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
                                    Units.degreesToRadians(-50+4)))));
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
                            Units.inchesToMeters(9.286)),
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

    public void stopDrive() {
        SwerveRequest.ApplyFieldSpeeds m_drive = new SwerveRequest.ApplyFieldSpeeds();
        drivetrain.setControl(m_drive.withSpeeds(new ChassisSpeeds(0,0,0)));
        System.out.println("STOP DRIVE");
    }

    private Command driveWithJoystick() {
        return drivetrain.applyRequest(
                () -> drive
                        .withVelocityX(
                                -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(
                                -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(
                                -joystick.getRightX()
                                        * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
    }
    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                driveWithJoystick());

        //joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.rightStick().whileTrue(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(
                                -joystick.getLeftX() * SlowSpeed)
                        .withVelocityY(
                                -joystick.getLeftY() * SlowSpeed)
                        .withRotationalRate(
                                -joystick.getRightX() * SlowAngularRate)));
        
        
    

        // joystick.y().whileTrue(s_Shooter.slowShoot()).onFalse(s_Shooter.stop());


        joystick.a().whileTrue(s_Climber.lower()).onFalse(s_Climber.stop());
        


        //joystick.b().onTrue(s_Elevator.goToL1()).onFalse(s_Elevator.stop());
        joystick.leftBumper().whileTrue(s_Elevator.goToL2().repeatedly());//.onFalse(s_Elevator.goToL1());
        joystick.rightBumper().whileTrue(s_Elevator.goToL3().repeatedly());//.onFalse(s_Elevator.goToL1());
        joystick.leftTrigger().whileTrue(s_Elevator.goToL4().repeatedly());//.onFalse(s_Elevator.goToL1());
        joystick.povUp().onTrue(s_Climber.climbPosition());
        joystick.povDown().whileTrue(Commands.sequence(s_Elevator.GroundIntake_Wait(), Commands.parallel(s_Elevator.GroundIntake().repeatedly(), Commands.sequence(s_Algae.groundIntake(), s_Algae.intake())))).onFalse(Commands.sequence(Commands.parallel(s_Algae.groundIntakeHome(),s_Elevator.L2_Wait()), s_Elevator.goToL1(), new WaitCommand(1),s_Algae.reZero()));
        //joystick.povDown().whileTrue(s_Elevator.moveDown()).onFalse(s_Elevator.stop());
        zeroController.x().onTrue(s_Elevator.reZero());

        // joystick.x().onTrue(s_Elevator.goToL3A()).onFalse(s_Elevator.goToL1());
        //joystick.a().onTrue(s_Elevator.goToL2A()).onFalse(s_Elevator.goToL1());
        joystick.povLeft().whileTrue(Commands.sequence(s_Elevator.goToL2A_wait(), Commands.parallel(s_Elevator.goToL2A().repeatedly(), Commands.sequence(s_Algae.extend(), s_Algae.intake())))).onFalse((s_Algae.home()));
        joystick.povRight().whileTrue(Commands.sequence(s_Elevator.goToL3A_wait(), Commands.parallel(s_Elevator.goToL3A().repeatedly(), Commands.sequence(s_Algae.extend(), s_Algae.intake())))).onFalse((s_Algae.home()));
        joystick.rightStick().onTrue(s_Algae.shoot()).onFalse(s_Algae.stopShooter());
        //joystick.back().whileTrue(s_Elevator.goToL1Intake().repeatedly().alongWith(s_Algae.intakeL1().andThen(s_Algae.shoot()))).onFalse(s_Algae.holdCoral().alongWith(s_Elevator.goToL1Shoot().repeatedly()));
        joystick.leftStick().whileTrue(s_Algae.shootL1low().andThen(s_Algae.slowShoot().andThen(s_Elevator.goToL1Shoot().repeatedly()))).onFalse(s_Algae.home().andThen(s_Algae.stopShooter().andThen(new WaitCommand(5).alongWith(s_Elevator.goToL1()))));
        //joystick.back().whileTrue(s_Elevator.goToDCMPL4().repeatedly());
       // joystick.leftStick().onTrue(s_Algae.intake()).onFalse(s_Algae.stopShooter());
        zeroController.a().onTrue(s_Climber.reZero());
        zeroController.b().onTrue(s_Algae.reZero());


        // shooter.shoot reneables the drivetrain BB
        // joystick.rightTrigger().onTrue(
        //         s_Shooter.shoot()
        // );
        // .onFalse(Commands.sequence(
        //         new WaitCommand(.5), 
        //         Commands.parallel(
        //                                 driveWithJoystick(),
        //                                 s_Shooter.stop()
        //                         )
        //         )
        // );

        joystick.rightTrigger().onTrue(new ConditionalCommand(
                s_Shooter.shoot(), 
                s_Shooter.slowShoot(), 
                () -> s_Elevator.isAtL4()
        )).onFalse(Commands.sequence(new WaitCommand(.5), Commands.parallel(driveWithJoystick(), s_Shooter.stop())));



        // auto align right
        // positive moves right for second param of translation
        joystick.b().whileTrue(Commands.parallel(
                        new DriveToPose(drivetrain, new Transform2d(Units.inchesToMeters(-33.5/2-1.5), Units.inchesToMeters(0.5+2.25+.25), Rotation2d.fromDegrees(-5)), joystick, 8), 
                        s_Shooter.shoot()
                ))
                .onFalse(new InstantCommand(()->joystick.setRumble(RumbleType.kBothRumble, 0))
                .andThen(Commands.parallel(s_Shooter.stop(), new RunCommand(() -> stopDrive(), drivetrain).repeatedly())));

        // stop the robot override
        joystick.button(7).whileTrue(new RunCommand(()-> {System.out.println("STOP");stopDrive(); }, drivetrain));

        // auto align left
        joystick.x().whileTrue(Commands.parallel(new DriveToPose(drivetrain,
                new Transform2d(Units.inchesToMeters(-33.5/2+0.75), Units.inchesToMeters(13.5+2.25),
                new Rotation2d()), joystick, 6),s_Shooter.shoot())).onFalse(new InstantCommand(()->joystick.setRumble(RumbleType.kBothRumble, 0))
                .andThen(Commands.parallel(s_Shooter.stop(), new RunCommand(() -> stopDrive(), drivetrain).repeatedly())));

            
        joystick.y().whileTrue(new AutoClimbCommand(joystick,drivetrain));                 
        
        
        // reset the field-centric heading on left bumper press
        joystick.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }



    public Command getAutonomousCommand() {
        // return new PathPlannerAuto("cut");
        return m_chooser.getSelected();
    }


}
