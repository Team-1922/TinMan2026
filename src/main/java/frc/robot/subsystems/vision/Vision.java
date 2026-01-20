// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    public Vision(VisionIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        setTagStrings();

        Logger.processInputs("Vision", inputs);
    }

    public boolean hasTag() {
        return inputs.hasTag;
    }

    public double getTx() {
        return inputs.tx;
    }

    public OptionalDouble getDistanceToTag(int targetId) {
        for (int i = 0; i < inputs.tagIds.length; i++) {
            if (inputs.tagIds[i] == targetId) {
                return OptionalDouble.of(inputs.tagDistances[i]);
            }
        }
        return OptionalDouble.empty();
    }

    public OptionalDouble getTxncForTag(int targetId) {
        for (int i = 0; i < inputs.tagIds.length; i++) {
            if (inputs.tagIds[i] == targetId) {
                return OptionalDouble.of(inputs.tagTxncs[i]);
            }
        }
        return OptionalDouble.empty();
    }

    private void setTagStrings() {
        inputs.tagStrings = new String[inputs.tagIds.length];
        
        for (int i = 0; i < inputs.tagIds.length; i++) {
            inputs.tagStrings[i] = String.format(
                    "ID: %-6d Distance: %-10.2f Txnc: %-10.2f",
                    inputs.tagIds[i],
                    inputs.tagDistances[i],
                    inputs.tagTxncs[i]);
        }        
    }
}
