/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Add your docs here.
 */
public class GetCargoCommand extends RamseteCommand {

    public GetCargoCommand(Trajectory trajectory, Supplier<Pose2d> pose, RamseteController follower,
            DifferentialDriveKinematics kinematics, BiConsumer<Double, Double> outputMetersPerSecond,
            Subsystem[] requirements) {
        super(trajectory, pose, follower, kinematics, outputMetersPerSecond, requirements);
    }
}
