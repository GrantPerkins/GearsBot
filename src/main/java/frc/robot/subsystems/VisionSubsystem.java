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
  /**
   * Represents a detected cargo from the Coral
   */
  public class Cargo {
    double distance;
    double x_offset;

    /**
     * Holds the data determined by Coral
     * 
     * @param heading
     * @param distance
     */
    public Cargo(double[] box) {
      this.distance = 231.13 * Math.pow(box[3] - box[1], -1.303);
      this.x_offset = (160 - ((box[0] + box[2]) / 2)) / (((box[3] - box[1]) / 13.0) / 39.37);
    }
  }

  /**
   * Represents a detected hatch from the Coral
   */
  public class Hatch {
    double distance;
    double x_offset;

    /**
     * Holds the data determined by Coral
     * 
     * @param name
     * @param heading
     * @param distance
     */
    public Hatch(double[] box) {
      this.distance = 289.67 * Math.pow(box[3] - box[1], -1.131);
      this.x_offset = (160 - ((box[0] + box[2]) / 2)) / (((box[3] - box[1]) / 19.5) / 39.37);
    }
  }

  NetworkTable table;
  int totalObjects;
  Cargo[] cargo;
  Hatch[] hatches;
  int totalCargo, totalHatches;
  private String[] classes;
  private double[] boxes, box;

  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("ML");
  }

  /**
   * Periodically updates the list of detected objects with the data found on
   * NetworkTables Also creates array of cargo and their relative position.
   */
  @Override
  public void periodic() {
    totalObjects = (int) table.getEntry("nb_objects").getNumber(0);
    totalCargo = 0;
    totalHatches = 0;
    classes = table.getEntry("object_classes").getStringArray(new String[totalObjects]);
    for (String s : classes) {
      if (s.equals("Cargo"))
        totalCargo++;
      if (s.equals("Hatchcover"))
        totalHatches++;
    }
    cargo = new Cargo[totalCargo];
    hatches = new Hatch[totalHatches];
    boxes = table.getEntry("boxes").getDoubleArray(new double[4 * totalObjects]);
    for (int i = 0; i < totalObjects; i += 4) {
      for (int j = 0; j < 4; j++) {
        box[j] = boxes[i + j];
      }
      if (classes[i].equals("Cargo"))
        cargo[i] = new Cargo(box);
      if (classes[i].equals("Hatch"))
        hatches[i] = new Hatch(box);
    }
  }
}
