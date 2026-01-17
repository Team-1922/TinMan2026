// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tuning;

import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Elastic;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class Tuner {
    double speed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final NetworkTableEntry metersPerSecond;

    public Tuner() {
        var table = NetworkTableInstance.getDefault().getTable("Tuner");
        metersPerSecond = table.getEntry("MetersPerSecond");
        metersPerSecond.setDouble(0);
    }
    
    public void periodic(){
        SmartDashboard.putNumber("Speed", speed);
      
    }
}
