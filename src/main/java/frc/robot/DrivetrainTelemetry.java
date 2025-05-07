package frc.robot;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

public class DrivetrainTelemetry {

	/**
	 * Accept the swerve drive state and telemeterize it to SmartDashboard and
	 * SignalLogger.
	 */
	public void telemeterize(SwerveDriveState state) {

		/* Telemeterize the swerve drive state */
		Logger.recordOutput("DriveState/Pose", state.Pose);
		Logger.recordOutput("DriveState/Speeds", state.Speeds);
		Logger.recordOutput("DriveState/ModuleStates", state.ModuleStates);
		Logger.recordOutput("DriveState/ModuleTargets", state.ModuleTargets);
		Logger.recordOutput("DriveState/ModulePositions", state.ModulePositions);
		Logger.recordOutput("DriveState/Timestamp", state.Timestamp);
		Logger.recordOutput("DriveState/OdometryPeriod", 1.0 / state.OdometryPeriod);
	}
}
