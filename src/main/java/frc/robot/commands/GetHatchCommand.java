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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.Hatch;

public class GetHatchCommand extends CommandBase{
  private VisionSubsystem vision;
  private DriveSubsystem drive;
  public GetHatchCommand(VisionSubsystem vision, DriveSubsystem drive) {
    super();
    this.vision = vision;
    this.drive = drive;
  }

  @Override
  public void initialize() {
    if (vision.totalHatches > 0) {
      Hatch hatch = vision.hatches[0];
      TrajectoryConfig config = new TrajectoryConfig(0.25, 1)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(Constants.DriveConstants.kDriveKinematics);

      // An example trajectory to follow. All units in meters.
      Trajectory hatchTrajectory = TrajectoryGenerator.generateTrajectory(
          List.of(new Pose2d(0, 0, new Rotation2d(0)),
                  new Pose2d(hatch.xOffset, hatch.distance, new Rotation2d(0))),
          // Pass config
          config);
      System.out.println(hatch.xOffset+" "+hatch.distance);

      RamseteCommand ramseteCommand = new RamseteCommand(hatchTrajectory,
          drive::getPose,
          new RamseteController(2, 0.7),
          Constants.DriveConstants.kFeedforward,
          Constants.DriveConstants.kDriveKinematics,
          drive::getWheelSpeeds,
          new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
          new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
          drive::tankDriveVolts,
          drive);
      Command command = ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
      command.schedule();
    }
  }
}
