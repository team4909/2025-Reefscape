package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class ReefBranchAlign extends Command {

    private final CommandSwerveDrivetrain m_drivetrain;
    private final ProfiledPIDController m_translationController, m_thetaController;
    private Translation2d m_lastSetpointTranslation;
    private Pose2d m_goalPose;
    private SwerveRequest.ApplyRobotSpeeds m_drive;
    private Transform2d m_shift;
    private DoubleSupplier m_joystickInput;
    private Pose2d m_lastPose;
  
  
    public ReefBranchAlign(CommandSwerveDrivetrain drivetrain, Transform2d shift, DoubleSupplier joystickXInput) {
      m_shift = shift;
      m_lastPose = new Pose2d();

      m_joystickInput = joystickXInput;

      m_translationController =
          new ProfiledPIDController(4.0, 0.0, 0.1, new TrapezoidProfile.Constraints(7, 7));
      m_thetaController =
          new ProfiledPIDController(
              5.0, 0.0, 0.1, new TrapezoidProfile.Constraints(1 * Math.PI, 1 * Math.PI));
      this.m_drivetrain = drivetrain;
  
      m_drive = new SwerveRequest.ApplyRobotSpeeds()
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
      .withSteerRequestType(SteerRequestType.MotionMagicExpo);
  
      addRequirements(drivetrain);
    }

  @Override
  public void initialize() {
    m_goalPose = m_drivetrain.findClosestNode().transformBy(m_shift);
    Pose2d initialPose = m_drivetrain.getState().Pose;

    m_translationController.setTolerance(0.1);
    m_thetaController.setTolerance(Units.degreesToRadians(3.0));
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    ChassisSpeeds fieldVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(m_drivetrain.getState().Speeds, initialPose.getRotation());

    m_translationController.reset(
        m_goalPose.relativeTo(initialPose).getTranslation().getY(),
        // Math.abs(initialPose.getTranslation().getY() - m_goalPose.getTranslation().getY()),
        // Math.min(0.0, m_drivetrain.getState().Speeds.vyMetersPerSecond));
        Math.min(
            0.0,
            -new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond)
                .rotateBy(
                    m_goalPose
                        .getTranslation()
                        .minus(initialPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
                
    m_thetaController.reset(
        initialPose.getRotation().getRadians(),
        fieldVelocity.omegaRadiansPerSecond);
    m_lastSetpointTranslation = initialPose.getTranslation();
  }

  @Override
  public void execute() {
    Pose2d currentPose = m_drivetrain.getState().Pose;
    double distanceToGoalPose = m_goalPose.relativeTo(currentPose).getTranslation().getY();
    // double yVelocity = m_translationController.calculate(distanceToGoalPose, 0);
    // double thetaVelocity = m_thetaController.calculate(currentPose.getRotation().getRadians(), m_goalPose.getRotation().getRadians());


    double ffScaler = MathUtil.clamp((distanceToGoalPose - 0.2) / (0.8 - 0.2), 0.0, 1.0);

    m_translationController.calculate(
        Math.abs(m_lastSetpointTranslation.getY() - m_goalPose.getX()),
        m_translationController.getSetpoint().velocity);//ROBO RELATIVE

    // double driveVelocity =
    //     m_translationController.getSetpoint().velocity * ffScaler
    //         + m_translationController.calculate(distanceToGoalPose, 0.0);

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

    double thetaVelocity = //m_thetaController.calculate(currentPose.getRotation().getRadians(), m_goalPose.getRotation().getRadians());
        m_thetaController.getSetpoint().velocity * ffScaler + m_thetaController.calculate(
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

    final ChassisSpeeds CS = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(m_joystickInput.getAsDouble(), driveVelocity.getY(), thetaVelocity), currentPose.getRotation());//new ChassisSpeeds(m_joystickInput.getAsDouble(), driveVelocity, thetaVelocity);

    m_drivetrain.setControl(m_drive.withSpeeds((new ChassisSpeeds(m_joystickInput.getAsDouble(), CS.vyMetersPerSecond, CS.omegaRadiansPerSecond))));

    Logger.recordOutput("Drivetrain/DriveToPose/ChassisSpeeds", CS);
    Logger.recordOutput("Drivetrain/DriveToPose/DistanceToGoalPose", distanceToGoalPose);
    Logger.recordOutput("Drivetrain/DriveToPose/Goal", m_goalPose);

    m_lastPose = currentPose;
  }
}