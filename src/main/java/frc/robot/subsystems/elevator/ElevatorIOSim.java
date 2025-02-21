package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class ElevatorIOSim implements ElevatorIO{
  private ElevatorSim elevatorSim;
  private LoggedMechanism2d elevatorMech = new LoggedMechanism2d(2, 2);
  private LoggedMechanismRoot2d root = elevatorMech.getRoot("elevatorRoot", 1, 0);
  private LoggedMechanismLigament2d elevatorModel = root.append(new LoggedMechanismLigament2d("Elevator", Units.inchesToMeters(ElevatorConstants.kL1), 90));

  private double volts;
  private double goalHeight;
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(80, 160);

  ProfiledPIDController profiledPIDController = new ProfiledPIDController(40, 0, 0.1, constraints);

  ElevatorFeedforward elevatorFF =
      new ElevatorFeedforward(0, 0, 0);

  public ElevatorIOSim() {
    Logger.recordOutput("Elevator Sim", elevatorMech);
    elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(2),
            12,
            Units.lbsToKilograms(10),
            Units.inchesToMeters(1.751),
            Units.inchesToMeters(ElevatorConstants.kL1),
            Units.inchesToMeters(ElevatorConstants.kL4),
            true,
            Units.inchesToMeters(ElevatorConstants.kL1));

    volts = 0;
    goalHeight = ElevatorConstants.kL1;
  }

  @Override
  public void updateInputs(ElevatorIOInputs elevatorInputs) {
    elevatorModel.setLength(elevatorSim.getPositionMeters());
    Logger.recordOutput("Elevator Sim", elevatorMech);
    elevatorSim.update(0.02);
    elevatorInputs.leftMotorConnected = true;
    elevatorInputs.rightMotorConnected = true;
    elevatorInputs.leftMotorVoltage = volts;
    elevatorInputs.rightMotorVoltage = volts;
    elevatorInputs.motorPosition = elevatorSim.getPositionMeters();
    elevatorInputs.goalPosition = goalHeight;
    elevatorInputs.velocity = elevatorSim.getVelocityMetersPerSecond();

    double calculatedVolts =
        profiledPIDController.calculate(
                elevatorSim.getPositionMeters(), Units.inchesToMeters(goalHeight))
            + elevatorFF.calculate(profiledPIDController.getSetpoint().velocity);
    setVoltage(calculatedVolts);
  }

  @Override
  public void setVoltage(double voltage) {
    volts = voltage;
    elevatorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void gotosetpoint(double setpoint, double gearRatio) {
    double calculatedVolts =
        profiledPIDController.calculate(
                elevatorSim.getPositionMeters(), Units.inchesToMeters(setpoint*gearRatio))
            + elevatorFF.calculate(profiledPIDController.getSetpoint().velocity);
    setVoltage(calculatedVolts);
    goalHeight = setpoint*gearRatio;
  }
}