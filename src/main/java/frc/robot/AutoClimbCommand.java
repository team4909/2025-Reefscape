// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Vision.VisionConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveToFieldPose;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoClimbCommand extends Command {

  private Pose2d blueStartPose = new Pose2d(7.495, 5.026, Rotation2d.fromDegrees(-90));
  private Pose2d blueFarPose = new Pose2d(8.9, 5.026, Rotation2d.fromDegrees(-90));
  private Pose2d blueClimbPose = new Pose2d(8.2, 5.026, Rotation2d.fromDegrees(-90));

  // var goToClimbStartPose = new DriveToFieldPose(drivetrain, startPose,
  // joystick, 3);

  // var goToClimbEndPose = new DriveToFieldPose(drivetrain, endPose, joystick,
  // .5);

  // var goToClimbMidPose = new DriveToFieldPose(drivetrain, ClimbMidPose,
  // joystick, 3);

  public boolean isBlueAlliance() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      return alliance.get() == DriverStation.Alliance.Blue;
    }
    return false;
  }

  public static Pose2d fieldCenter = new Pose2d(VisionConstants.aprilTagLayout.getFieldLength() / 2,
      VisionConstants.aprilTagLayout.getFieldWidth() / 2, Rotation2d.fromDegrees(0));

  public static Pose2d rotatePoseAboutFieldCenter(Pose2d pose) {

    var fieldCentricBluePose = pose.relativeTo(fieldCenter);
    var fieldCentriclRedPose = fieldCentricBluePose.rotateBy(Rotation2d.fromDegrees(180));
    var redPose = fieldCentriclRedPose.relativeTo(new Pose2d(-VisionConstants.aprilTagLayout.getFieldLength() / 2,
        -VisionConstants.aprilTagLayout.getFieldWidth() / 2, Rotation2d.fromDegrees(0)));
    return redPose;
  }

  public Command driveCommand;


  private CommandXboxController m_js;
  private CommandSwerveDrivetrain m_drivetrain;

  public AutoClimbCommand(CommandXboxController js, CommandSwerveDrivetrain drivetrain) {
    m_js = js;
    m_drivetrain = drivetrain;
  }

  public boolean poseEqualsPoseWithDelta(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation()) < Units.inchesToMeters(2)
                && (a.getRotation().getDegrees() - b.getRotation().getDegrees() < 5);
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("isBlue " + isBlueAlliance());

    Pose2d startPose;
    Pose2d farPose;
    Pose2d climbPose;
    if (isBlueAlliance()) {
      startPose = blueStartPose;
      farPose = blueFarPose;
      climbPose = blueClimbPose;
    } else {
      startPose = rotatePoseAboutFieldCenter(blueStartPose);
      farPose = rotatePoseAboutFieldCenter(blueFarPose);
      climbPose = rotatePoseAboutFieldCenter(blueClimbPose);
    }

    Command startPoseCommand = new DriveToFieldPose(m_drivetrain, startPose, m_js, 3);
    Command farPoseCommand = new DriveToFieldPose(m_drivetrain, farPose, m_js, .5);
    Command climbPoseCommand = new DriveToFieldPose(m_drivetrain, climbPose, m_js, 1.5);

    if (!poseEqualsPoseWithDelta(m_drivetrain.getState().Pose, startPose ) && !poseEqualsPoseWithDelta(m_drivetrain.getState().Pose, climbPose )) {
      driveCommand = startPoseCommand;
    } else if(poseEqualsPoseWithDelta(m_drivetrain.getState().Pose, startPose)) {
      driveCommand = farPoseCommand;
    } else {
      driveCommand = climbPoseCommand;
    }


    // new ConditionalCommand(goToClimbStartPose,
    // new ConditionalCommand(goToClimbEndPose, goToClimbMidPose,()->
    // !poseEqualsPoseWithDelta(drivetrain.getState().Pose, endPose))
    // , ()-> !poseEqualsPoseWithDelta(drivetrain.getState().Pose, startPose ) &&
    //  ))
    driveCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveCommand.execute();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
