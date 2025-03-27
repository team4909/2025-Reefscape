// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision.VisionConstants;
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

    // Translation2d relativeTranslation =
    // pose.getTranslation().minus(fieldCenter.getTranslation());
    // Rotation2d relativeRotation =
    // pose.getRotation().minus(fieldCenter.getRotation());

    // Translation2d rotatedTranslation =
    // relativeTranslation.rotateBy(fieldCenter.getRotation());
    // Rotation2d rotatedRotation =
    // relativeRotation.rotateBy(fieldCenter.getRotation());

    // return new Pose2d(rotatedTranslation.plus(fieldCenter.getTranslation()),
    // rotatedRotation.plus(fieldCenter.getRotation()));
  }

  public AutoClimbCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("isBlue " + isBlueAlliance());

    if (isBlueAlliance()) {
      System.out.println("blueStartPose " + blueStartPose);
    } else {
      System.out.println("redStartPose " + rotatePoseAboutFieldCenter(blueStartPose));
    }

    // new ConditionalCommand(goToClimbStartPose,
    // new ConditionalCommand(goToClimbEndPose, goToClimbMidPose,()->
    // !poseEqualsPoseWithDelta(drivetrain.getState().Pose, endPose))
    // , ()-> !poseEqualsPoseWithDelta(drivetrain.getState().Pose, startPose ) &&
    // !poseEqualsPoseWithDelta(drivetrain.getState().Pose, endPose ))
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
