package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;


import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {

  private final CommandSwerveDrivetrain m_drivetrain;
  private final ProfiledPIDController m_translationController, m_thetaController;
  private Translation2d m_lastSetpointTranslation;
  private Pose2d m_goalPose;
  private SwerveRequest.ApplyFieldSpeeds m_drive;
  private CommandXboxController m_controller;
  Transform2d m_shift;


  public DriveToPose(CommandSwerveDrivetrain drivetrain, Transform2d shift, CommandXboxController controller) {
    m_shift = shift;
    m_controller = controller;
    m_translationController =
        new ProfiledPIDController(6.0, 0.0, 0.0, new TrapezoidProfile.Constraints(3, 4));
    m_thetaController =
        new ProfiledPIDController(
            4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(100 * Math.PI, 100 * Math.PI));
    this.m_drivetrain = drivetrain;

    m_drive = new SwerveRequest.ApplyFieldSpeeds()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_goalPose = m_drivetrain.findClosestNode().transformBy(m_shift);
    Pose2d initialPose = m_drivetrain.getState().Pose;

    m_translationController.setTolerance(0.01);
    m_thetaController.setTolerance(Units.degreesToRadians(1.0));

    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    ChassisSpeeds robotVelocity = getFieldRelativeChassisSpeeds(m_drivetrain.getState().Speeds, initialPose);

    m_translationController.reset(
        initialPose.getTranslation().getDistance(m_goalPose.getTranslation()),
        
        Math.min(
            0.0,
            -new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond)
                .rotateBy(
                    m_goalPose
                        .getTranslation()
                        .minus(initialPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));

    m_thetaController.reset(
        initialPose.getRotation().getRadians(),
        robotVelocity.omegaRadiansPerSecond);
    m_lastSetpointTranslation = initialPose.getTranslation();

    
  }

  @Override
  public void execute() {
    Pose2d currentPose = m_drivetrain.getState().Pose;

    double distanceToGoalPose =
        currentPose.getTranslation().getDistance(m_goalPose.getTranslation());
    
        if (distanceToGoalPose<Units.inchesToMeters(1)){
            m_controller.setRumble(RumbleType.kBothRumble, 1);
        }

    double ffScaler = MathUtil.clamp((distanceToGoalPose - 0.2) / (0.8 - 0.2), 0.0, 1.0);

    m_translationController.reset(
        m_lastSetpointTranslation.getDistance(m_goalPose.getTranslation()),
        m_translationController.getSetpoint().velocity);//ROBO RELATIVE

    double driveVelocityScalar =
        m_translationController.getSetpoint().velocity * ffScaler
            + m_translationController.calculate(distanceToGoalPose, 0.0);

    if (distanceToGoalPose < m_translationController.getPositionTolerance())
      driveVelocityScalar = 0.0;
      
    m_lastSetpointTranslation =
        new Pose2d(
                m_goalPose.getTranslation(),
                currentPose.getTranslation().minus(m_goalPose.getTranslation()).getAngle())
            .transformBy(
                new Transform2d(
                    new Translation2d(m_translationController.getSetpoint().position, 0.0),
                    new Rotation2d()))
            .getTranslation();

    double thetaVelocity =
        m_thetaController.getSetpoint().velocity * ffScaler
            + m_thetaController.calculate(
                currentPose.getRotation().getRadians(), m_goalPose.getRotation().getRadians());
    double thetaErrorAbsolute =
        Math.abs(currentPose.getRotation().minus(m_goalPose.getRotation()).getRadians());
    if (thetaErrorAbsolute < m_thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    Translation2d driveVelocity =
        new Pose2d(
                new Translation2d(),
                currentPose.getTranslation().minus(m_goalPose.getTranslation()).getAngle())
            .transformBy(
                new Transform2d(new Translation2d(driveVelocityScalar, 0.0), new Rotation2d()))
            .getTranslation();

    final ChassisSpeeds CS = new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity);

    m_drivetrain.setControl(m_drive.withSpeeds(CS));

    Logger.recordOutput("Drivetrain/DriveToPose/ChassisSpeeds", CS);

    Logger.recordOutput("Drivetrain/DriveToPose/DistanceToGoalPose", distanceToGoalPose);
    Logger.recordOutput(
        "Drivetrain/DriveToPose/Setpoint",
        new Pose2d(
            m_lastSetpointTranslation, new Rotation2d(m_thetaController.getSetpoint().position)));
    Logger.recordOutput("Drivetrain/DriveToPose/Goal", m_goalPose);
  }

  public ChassisSpeeds getFieldRelativeChassisSpeeds(ChassisSpeeds roboSpeed, Pose2d pose) {
    return new ChassisSpeeds(
        roboSpeed.vxMetersPerSecond * pose.getRotation().getCos()
                    - roboSpeed.vyMetersPerSecond * pose.getRotation().getSin(),
                    roboSpeed.vyMetersPerSecond * pose.getRotation().getCos()
                    + roboSpeed.vxMetersPerSecond * pose.getRotation().getSin(),
                    roboSpeed.omegaRadiansPerSecond);
}
}