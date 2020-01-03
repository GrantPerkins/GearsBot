package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Command that turns the robot to face the first detected hatch
 */
public class TurnToHatchCommand extends PIDCommand {
  private VisionSubsystem visionSubsystem;
  private DriveSubsystem driveSubsystem;

  /**
   * PIDCommand uses relative hatch angle at time of initialization, not construction.
   * Turn is based on gyro.
   * @param driveSubsystem the drive subsystem
   * @param visionSubsystem the vision subsystem
   */
  public TurnToHatchCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    super(
        // Tune these values for your chassis
        new PIDController(1, 0, 0),
        // Gets the heading of the robot in radians as PID input
        () -> driveSubsystem.getGyroAngle().getRadians(),
        // 0 setpoint at construction, immediately overwritten at time of init
        0,
        // Turns with output
        (double value) -> driveSubsystem.drive(0, value, true),
        // Required subsystems
        driveSubsystem,
        visionSubsystem
    );
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    // Set tolerance for PID
    getController().setTolerance(10, 5);
    // Using gyro heading in Radians, so continuous input
    getController().enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Sets setpoint at init
   */
  @Override
  public void initialize() {
    super.initialize();
    getController().setSetpoint(getHatchAngle());
  }

  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }

  /**
   * Gets the angle of the hatch if there is one
   * @return angle of hatch
   */
  public double getHatchAngle() {
    if (visionSubsystem.totalHatches > 0) {
      return visionSubsystem.hatches[0].getAngle();
    }
    // return current drivetrain angle if no hatch detected, so no turning
    return driveSubsystem.getGyroAngle().getRadians();
  }
}
