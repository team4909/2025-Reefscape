// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.*;

public class Candle extends SubsystemBase {
    private final CANdle m_candle;

    public Candle(){
        m_candle = new CANdle(100);
        // setDefaultCommand(this.setColor(200,0,0));
    }

    public Command setColor(int r, int g, int b){
        return this.runOnce(() -> m_candle.setLEDs(r, g, b));
    }
}