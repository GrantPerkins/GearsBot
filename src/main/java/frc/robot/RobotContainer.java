/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmMiddleCommand;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.CloseClawCommand;
import frc.robot.commands.OpenClawCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.GamepieceRetriever;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final PowerDistributionPanel pdp = new PowerDistributionPanel();

  private final ArmSubsystem arm = new ArmSubsystem();
  private final ClawSubsystem claw = new ClawSubsystem();
  private final DriveSubsystem drive = new DriveSubsystem();
  private final VisionSubsystem vision = new VisionSubsystem();
  private final GamepieceRetriever retriever = new GamepieceRetriever(vision, drive);

  private final XboxController gamepad = new XboxController(0);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    drive.setDefaultCommand(new TeleopDriveCommand(drive,
        () -> -1 * gamepad.getY(GenericHID.Hand.kLeft),
        () -> gamepad.getX(GenericHID.Hand.kRight),
        () -> gamepad.getStickButton(GenericHID.Hand.kRight)));

    SmartDashboard.putData(pdp);

    SmartDashboard.putData(arm);
    SmartDashboard.putData(claw);
    SmartDashboard.putData(drive);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final Button a = new JoystickButton(gamepad, XboxController.Button.kA.value);
    final Button b = new JoystickButton(gamepad, XboxController.Button.kB.value);
    final Button dpadUp = new POVButton(gamepad, 0);
    final Button dpadRight = new POVButton(gamepad, 90);
    final Button dpadDown = new POVButton(gamepad, 180);
    final Button rightBumper = new JoystickButton(gamepad, XboxController.Button.kBumperRight.value);
    final Button leftBumper = new JoystickButton(gamepad, XboxController.Button.kBumperLeft.value);

    a.whenPressed(new OpenClawCommand(claw));
    b.whenPressed(new CloseClawCommand(claw));
    dpadUp.whenPressed(new ArmUpCommand(arm));
    dpadRight.whenPressed(new ArmMiddleCommand(arm));
    dpadDown.whenPressed(new ArmDownCommand(arm));
    rightBumper.whenPressed(retriever.getCargoCommand());
    leftBumper.whenPressed(retriever.getHatchCommand());
  }

  TrajectoryConfig config = new TrajectoryConfig(0.25, 1)
    .setKinematics(Constants.DriveConstants.kDriveKinematics);

  Trajectory trajectory1 = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(0.0, 0, new Rotation2d(0)),
        new Pose2d(0.75, 0, new Rotation2d(0))),
      config);
  Trajectory trajectory2 = TrajectoryGenerator.generateTrajectory(
      List.of(
        new Pose2d(0.75, 0, new Rotation2d(0)),
        new Pose2d(1.65, 0, new Rotation2d(0))),
      config);

  public Command getAutonomousCommand() {
    return getSplineCommand();
    // return new SequentialCommandGroup(
    //   new ParallelCommandGroup(
    //     new ArmDownCommand(arm),
    //     new OpenClawCommand(claw),
    //     new FollowDriveTrajectoryCommand(drive, trajectory1)
    //   ),
    //   new CloseClawCommand(claw),
    //   new ParallelCommandGroup(
    //     new ArmUpCommand(arm),
    //     new FollowDriveTrajectoryCommand(drive, trajectory2)
    //   ),
    //   new OpenClawCommand(claw)
    // );
  }

  public Command getSplineCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(0.25, 1)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(0.5, 0.5),
            new Translation2d(1.0, 0.0),
            new Translation2d(1.5, 0.5),
            new Translation2d(2.0, 0.0),
            new Translation2d(2.5, 0.5)
        ),
        // End 3.5 meters straight ahead of where we started, facing forward
        new Pose2d(3.5, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        drive::getPose,
        new RamseteController(2, 0.7),
        Constants.DriveConstants.kFeedforward,
        Constants.DriveConstants.kDriveKinematics,
        drive::getWheelSpeeds,
        new PIDController(0, 0, 0),
        new PIDController(0, 0, 0),
        drive::tankDriveVolts,
        drive
    );

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> drive.tankDriveVolts(0, 0));
  }
}
