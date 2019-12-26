/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Cargo;
import frc.robot.subsystems.VisionSubsystem.Hatch;

public class GamepieceRetriever {
  DriveSubsystem drive;
  VisionSubsystem vision;

  public GamepieceRetriever(VisionSubsystem vision, DriveSubsystem drive) {
    this.vision = vision;
    this.drive = drive;

  }

  /**
   * Drives to nearest cargo, if there is one.
   * @return a Ramsete command that drives to nearest cargo
   */
  public Command getCargoCommand() {
    if (vision.totalCargo > 0) {
      Cargo cargo = vision.cargo[0];
      TrajectoryConfig config = new TrajectoryConfig(0.25, 1)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(Constants.DriveConstants.kDriveKinematics);

      // An example trajectory to follow. All units in meters.
      Trajectory cargoTrajectory = TrajectoryGenerator.generateTrajectory(
          List.of(new Pose2d(0, 0, new Rotation2d(0)),
                  new Pose2d(cargo.x_offset, cargo.distance, new Rotation2d(0))),
          // Pass config
          config);

      RamseteCommand ramseteCommand = new RamseteCommand(
          cargoTrajectory,
          drive::getPose, 
          new RamseteController(2, 0.7),
          Constants.DriveConstants.kFeedforward,
          Constants.DriveConstants.kDriveKinematics,
          drive::getWheelSpeeds,
          new PIDController(0, 0, 0),
          new PIDController(0, 0, 0),
          drive::tankDriveVolts,
          drive);
      return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
    }
    return new WaitCommand(.1);
  }
    

  /**
   * Drives to nearest hatch, if there is one.
   * @return a Ramsete command that drives to nearest hatch
   */
  public Command getHatchCommand() {
    if (vision.totalHatches > 0) {
      Hatch hatch = vision.hatches[0];
      TrajectoryConfig config = new TrajectoryConfig(0.25, 1)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(Constants.DriveConstants.kDriveKinematics);

      // An example trajectory to follow. All units in meters.
      Trajectory hatchTrajectory = TrajectoryGenerator.generateTrajectory(
          List.of(new Pose2d(0, 0, new Rotation2d(0)),
                  new Pose2d(hatch.x_offset, hatch.distance, new Rotation2d(0))),
          // Pass config
          config);

      RamseteCommand ramseteCommand = new RamseteCommand(hatchTrajectory,
          drive::getPose,
          new RamseteController(2, 0.7),
          Constants.DriveConstants.kFeedforward,
          Constants.DriveConstants.kDriveKinematics,
          drive::getWheelSpeeds,
          new PIDController(0, 0, 0),
          new PIDController(0, 0, 0),
          drive::tankDriveVolts,
          drive);
      return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
    }
    return new WaitCommand(.1);
  }
}
