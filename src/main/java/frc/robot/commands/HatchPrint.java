/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;

public class HatchPrint extends InstantCommand {
    public HatchPrint(VisionSubsystem vision) {
        super(() -> System.out.println(HatchPrint.getMessage(vision)));
    }

    private static String getMessage(VisionSubsystem vision) {
        if (vision.hatches.length > 0)
            return String.format("Hatch at: %f, %f", vision.hatches[0].xOffset, vision.hatches[0].distance);
        return "No hatch.";
    }
}
