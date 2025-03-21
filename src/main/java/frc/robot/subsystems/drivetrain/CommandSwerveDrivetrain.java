package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.GeometryUtil;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import javax.sound.sampled.SourceDataLine;

import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private SwerveRequest.ApplyRobotSpeeds m_drive;

    // private List<Pose2d> scoringPoses = new ArrayList<>(12);
    // scoringPoses.add()

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private AprilTagFieldLayout aprilTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    private final List<Pose2d> alignPositions;
    private Pose2d[] centerFaces = new Pose2d[6];

    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices
     * themselves. If they need the devices, they can access them through getters in
     * the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        m_drive = new SwerveRequest.ApplyRobotSpeeds();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> this.getState().Pose,
                    this::resetPose,
                    this::getRobotRelativeSpeeds,
                    (speeds, feedforwards) -> setControl(m_drive.withSpeeds(speeds)),
                    // this.ChassisSpeeds(Translation2d.driveVelocity.getX(),
                    // Translation2d.driveVelocity.getY(), thetaVelocity),
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(5.0, 0.0, 0.0)),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    }, this);

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
        centerFaces[0] = new Pose2d(
                Units.inchesToMeters(144.003),
                Units.inchesToMeters(158.500),
                Rotation2d.fromDegrees(0))
                .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0), new Rotation2d()));
        centerFaces[1] = new Pose2d(
                Units.inchesToMeters(160.373),
                Units.inchesToMeters(186.857),
                Rotation2d.fromDegrees(-60))
                .transformBy(new Transform2d(new Translation2d(Units.inchesToMeters(13), 0), new Rotation2d()));
        centerFaces[2] = new Pose2d(
                Units.inchesToMeters(193.116),
                Units.inchesToMeters(186.858),
                Rotation2d.fromDegrees(-120))
                .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0), new Rotation2d()));
        centerFaces[3] = new Pose2d(
                Units.inchesToMeters(209.489),
                Units.inchesToMeters(158.502),
                Rotation2d.fromDegrees(180))
                .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0), new Rotation2d()));
        centerFaces[4] = new Pose2d(
                Units.inchesToMeters(193.118),
                Units.inchesToMeters(130.145),
                Rotation2d.fromDegrees(120))
                .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0), new Rotation2d()));
        centerFaces[5] = new Pose2d(
                Units.inchesToMeters(160.375),
                Units.inchesToMeters(130.144),
                Rotation2d.fromDegrees(60))
                .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0), new Rotation2d()));
        
        if (DriverStation.getAlliance().isPresent()) {
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
                for(int i = 0; i<centerFaces.length; i++){
                    centerFaces[i].getTranslation().rotateAround(new Translation2d(aprilTagLayout.getFieldLength()/2, aprilTagLayout.getFieldWidth()/2), Rotation2d.k180deg);
                    centerFaces[i].getRotation().rotateBy(Rotation2d.k180deg);
                }
            }
        }
        // alignPositions = Arrays.asList(centerFaces);
        alignPositions = new ArrayList<>();
        
        int[] a = {6,7,8,9,10,11, 17,18,19,20,21,22};
        for (var tag : aprilTagLayout.getTags()) {
            if (Arrays.stream(a).anyMatch(x -> x == tag.ID)) {
                alignPositions.add(new Pose2d(tag.pose.toPose2d().getTranslation(), tag.pose.toPose2d().getRotation().rotateBy(Rotation2d.k180deg)));
            }
        }
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return this.getKinematics().toChassisSpeeds(this.getState().ModuleStates);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices
     * themselves. If they need the devices, they can access them through getters in
     * the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to
     *                                0 Hz, this is 250 Hz on CAN FD, and 100 Hz on
     *                                CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    // public CommandSwerveDrivetrain(
    // SwerveDrivetrainConstants drivetrainConstants,
    // double odometryUpdateFrequency,
    // SwerveModuleConstants<?, ?, ?>... modules) {
    // super(drivetrainConstants, odometryUpdateFrequency, modules);
    // if (Utils.isSimulation()) {
    // startSimThread();
    // }
    // centerFaces[0] = new Pose2d(
    // Units.inchesToMeters(144.003),
    // Units.inchesToMeters(158.500),
    // Rotation2d.fromDegrees(0))
    // .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0),
    // new Rotation2d()));
    // centerFaces[1] = new Pose2d(
    // Units.inchesToMeters(160.373),
    // Units.inchesToMeters(186.857),
    // Rotation2d.fromDegrees(-60))
    // .transformBy(new Transform2d(new Translation2d(Units.inchesToMeters(13), 0),
    // new Rotation2d()));
    // centerFaces[2] = new Pose2d(
    // Units.inchesToMeters(193.116),
    // Units.inchesToMeters(186.858),
    // Rotation2d.fromDegrees(-120))
    // .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0),
    // new Rotation2d()));
    // centerFaces[3] = new Pose2d(
    // Units.inchesToMeters(209.489),
    // Units.inchesToMeters(158.502),
    // Rotation2d.fromDegrees(180))
    // .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0),
    // new Rotation2d()));
    // centerFaces[4] = new Pose2d(
    // Units.inchesToMeters(193.118),
    // Units.inchesToMeters(130.145),
    // Rotation2d.fromDegrees(120))
    // .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0),
    // new Rotation2d()));
    // centerFaces[5] = new Pose2d(
    // Units.inchesToMeters(160.375),
    // Units.inchesToMeters(130.144),
    // Rotation2d.fromDegrees(60))
    // .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0),
    // new Rotation2d()));

    // alignPositions = Arrays.asList(centerFaces);
    // System.out.println("Con 2");
    // }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct the devices
     * themselves. If they need the devices, they can access them through getters in
     * the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to
     *                                  0 Hz, this is 250 Hz on CAN FD, and 100 Hz
     *                                  on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation in the form
     *                                  [x, y, theta]ᵀ, with units in meters and
     *                                  radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation in the form [x, y,
     *                                  theta]ᵀ, with units in meters and radians
     * @param modules                   Constants for each specific module
     */
    // public CommandSwerveDrivetrain(
    // SwerveDrivetrainConstants drivetrainConstants,
    // double odometryUpdateFrequency,
    // Matrix<N3, N1> odometryStandardDeviation,
    // Matrix<N3, N1> visionStandardDeviation,
    // SwerveModuleConstants<?, ?, ?>... modules) {
    // super(
    // drivetrainConstants,
    // odometryUpdateFrequency,
    // odometryStandardDeviation,
    // visionStandardDeviation,
    // modules);
    // if (Utils.isSimulation()) {
    // startSimThread();
    // }
    // centerFaces[0] = new Pose2d(
    // Units.inchesToMeters(144.003),
    // Units.inchesToMeters(158.500),
    // Rotation2d.fromDegrees(0))
    // .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0),
    // new Rotation2d()));
    // centerFaces[1] = new Pose2d(
    // Units.inchesToMeters(160.373),
    // Units.inchesToMeters(186.857),
    // Rotation2d.fromDegrees(-60))
    // .transformBy(new Transform2d(new Translation2d(Units.inchesToMeters(13), 0),
    // new Rotation2d()));
    // centerFaces[2] = new Pose2d(
    // Units.inchesToMeters(193.116),
    // Units.inchesToMeters(186.858),
    // Rotation2d.fromDegrees(-120))
    // .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0),
    // new Rotation2d()));
    // centerFaces[3] = new Pose2d(
    // Units.inchesToMeters(209.489),
    // Units.inchesToMeters(158.502),
    // Rotation2d.fromDegrees(180))
    // .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0),
    // new Rotation2d()));
    // centerFaces[4] = new Pose2d(
    // Units.inchesToMeters(193.118),
    // Units.inchesToMeters(130.145),
    // Rotation2d.fromDegrees(120))
    // .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0),
    // new Rotation2d()));
    // centerFaces[5] = new Pose2d(
    // Units.inchesToMeters(160.375),
    // Units.inchesToMeters(130.144),
    // Rotation2d.fromDegrees(60))
    // .transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(13), 0),
    // new Rotation2d()));

    // alignPositions = Arrays.asList(centerFaces);
    // System.out.println("Con 3");
    // }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link
     * #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine specified
     * by {@link
     * #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance()
                    .ifPresent(
                            allianceColor -> {
                                setOperatorPerspectiveForward(
                                        allianceColor == Alliance.Red
                                                ? kRedAlliancePerspectiveRotation
                                                : kBlueAlliancePerspectiveRotation);
                                m_hasAppliedOperatorPerspective = true;
                            });
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(
                () -> {
                    final double currentTime = Utils.getCurrentTimeSeconds();
                    double deltaTime = currentTime - m_lastSimTime;
                    m_lastSimTime = currentTime;

                    /* use the measured time delta, get battery voltage from WPILib */
                    updateSimState(deltaTime, RobotController.getBatteryVoltage());
                });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Pose2d findClosestNode() {
        System.out.println("X");
        for (int i = alignPositions.size(); i < 6; i++) {
            Logger.recordOutput("Vision/ReefFaces" + i, alignPositions.get(i));
        }

        Pose2d currentPose = this.getState().Pose;
        Pose2d goal = currentPose.nearest(alignPositions);
        Logger.recordOutput("Vision/GoalPose", goal);
        return goal;
    }

    public void resetPose(Pose2d pose) {
        if (pose == null) {
            return;
        }
        super.resetPose(pose);
    }
}
