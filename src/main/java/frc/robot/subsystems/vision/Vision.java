// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

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
    }

    // Accessors for commands
    public boolean hasTag() {
        return inputs.hasTag;
    }

    public double getTx() {
        return inputs.tx;
    }

    public Double getDistanceToTag(int targetId) {
        for (int i = 0; i < inputs.tagIds.length; i++) {
            if (inputs.tagIds[i] == targetId) {
                return inputs.tagDistances[i];
            }
        }
        return 0.0;
    }

    public Double getTxncForTag(int targetId) {
        for (int i = 0; i < inputs.tagIds.length; i++) {
            if (inputs.tagIds[i] == targetId) {
                return inputs.tagTxncs[i];
            }
        }
        return 0.0;
    }
}
