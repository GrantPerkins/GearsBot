/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private NetworkTable table;
  private int totalObjects;
  public Cargo[] cargo = new Cargo[0];
  public Hatch[] hatches = new Hatch[0];
  public int totalCargo, totalHatches;
  private String[] classes;
  private double[] boxes, box;

  /**
   * Represents a detected cargo from the Coral
   */
  public class Cargo {
    public double distance;
    public double xOffset;

    /**
     * Holds the data determined by Coral
     *
     * @param box the array of points
     */
    public Cargo(double[] box) {
      this.distance = 231.13 * Math.pow(box[3] - box[1], -1.303);
      this.xOffset = (160 - ((box[0] + box[2]) / 2)) / (((box[3] - box[1]) / 13.0) * 39.37);
    }
  }

  /**
   * Represents a detected hatch from the Coral
   */
  public class Hatch {
    public double distance;
    public double xOffset;

    /**
     * Holds the data determined by Coral
     *
     * @param box the array of points
     */
    public Hatch(double[] box) {
      this.distance = 289.67 * Math.pow(box[3] - box[1], -1.131);
      this.xOffset = (160 - ((box[0] + box[2]) / 2)) / (((box[3] - box[1]) / 19.5) * 39.37);
    }
  }

  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("ML");
  }

  /**
   * Periodically updates the list of detected objects with the data found on
   * NetworkTables Also creates array of cargo and their relative position.
   */
  @Override
  public void periodic() {
    totalCargo = 0;
    totalHatches = 0;
    totalObjects = (int) table.getEntry("nb_objects").getDouble(0);
    classes = table.getEntry("object_classes").getStringArray(new String[totalObjects]);
    boxes = table.getEntry("boxes").getDoubleArray(new double[4 * totalObjects]);
    for (String s : classes) {
      if (s.equals("Cargo"))
        totalCargo++;
      if (s.equals("Hatchcover"))
        totalHatches++;
    }
    cargo = new Cargo[totalCargo];
    hatches = new Hatch[totalHatches];

    for (int i = 0; i < totalObjects; i += 4) {
      box = new double[4];
      for (int j = 0; j < 4; j++) {
        box[j] = boxes[i + j];
      }
      if (classes[i].equals("Cargo"))
        cargo[i] = new Cargo(box);
      if (classes[i].equals("Hatchcover"))
        hatches[i] = new Hatch(box);
    }
  }
}
