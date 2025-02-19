package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase {
    private ElevatorIO m_io;
    private ElevatorIOInputsAutoLogged m_inputs = new ElevatorIOInputsAutoLogged();

      public Elevator(ElevatorIO io) {
        m_io = io;
      }

      @Override
      public void periodic() {
        m_io.updateInputs(m_inputs);
        Logger.processInputs("Elevator", m_inputs);
      }
      

      //---------COMMANDS-------------
      public Command moveUp() {
        return this.runOnce(() -> m_io.setVoltage(3)).withName("UP");
      }
    
      public Command stop() {
        return this.runOnce(() -> m_io.setVoltage(0)).withName("Stop");
      }
    
      public Command moveDown() {
        return this.runOnce(() -> m_io.setVoltage(-3)).withName("Move Down");
      }

      public Command goToL1(){
        return this.runOnce(() -> {
            m_io.gotosetpoint(ElevatorConstants.kL1, ElevatorConstants.kGEAR_RATIO);
        }).withName("L1");
      }
      public Command goToL2(){
        return this.runOnce(() -> {
            m_io.gotosetpoint(ElevatorConstants.kL2, ElevatorConstants.kGEAR_RATIO);
        }).withName("L2");
      }
      public Command goToL3(){
        return this.runOnce(() -> {
            m_io.gotosetpoint(ElevatorConstants.kL3, ElevatorConstants.kGEAR_RATIO);
        }).withName("L3");
      }
      public Command goToL4(){
        return this.runOnce(() -> {
            m_io.gotosetpoint(ElevatorConstants.kL4, ElevatorConstants.kGEAR_RATIO);
        }).withName("L4");
      }
      public Command goToL3A(){
        return this.runOnce(() -> {
            m_io.gotosetpoint(ElevatorConstants.kL3A, ElevatorConstants.kGEAR_RATIO);
        }).withName("L3A");
      }
      public Command goToL2A(){
        return this.runOnce(() -> {
            m_io.gotosetpoint(ElevatorConstants.kL2A, ElevatorConstants.kGEAR_RATIO);
        }).withName("L2A");
      }

      public Command reZero(){
        return this.runOnce(() -> {
            m_io.setPosition(ElevatorConstants.kL1*ElevatorConstants.kGEAR_RATIO);
        }).withName("ReZero");
      }

      //--------------SYSID-------------
      private final SysIdRoutine m_SysIdRoutine = 
        new SysIdRoutine(
          new SysIdRoutine.Config(
            null, //1 volt a sec
            Volts.of(4), //constant 4 volts
            null, //10 sec timeout
            (state) -> SignalLogger.writeString("state", state.toString())
          ),
          new SysIdRoutine.Mechanism(
            (volts) -> m_io.setVoltage(volts.in(Volts)),
            null,
            this
          )
        );
      
      public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.quasistatic(direction);
      }
       
      public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_SysIdRoutine.dynamic(direction);
      }

      public Command runSysID(){
        return 
        this.runOnce(SignalLogger::start)
        .andThen(this.sysIdQuasistatic(SysIdRoutine.Direction.kForward))
        .until(()-> m_io.getPosition() == ElevatorConstants.kL4*ElevatorConstants.kGEAR_RATIO)
        .andThen(this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse))
        .until(() -> m_io.getPosition() == ElevatorConstants.kL1*ElevatorConstants.kGEAR_RATIO)
        .andThen(this.sysIdDynamic(SysIdRoutine.Direction.kForward))
        .until(()-> m_io.getPosition() == ElevatorConstants.kL4*ElevatorConstants.kGEAR_RATIO)
        .andThen(this.sysIdDynamic(SysIdRoutine.Direction.kReverse))
        .until(() -> m_io.getPosition() == ElevatorConstants.kL1*ElevatorConstants.kGEAR_RATIO)
        .finallyDo(SignalLogger::stop);
      }
}
