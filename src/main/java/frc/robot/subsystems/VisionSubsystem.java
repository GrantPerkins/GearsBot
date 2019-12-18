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
   * Represents a detected game object from the Coral
   */
  public class GameObject {
    String name;
    double heading;
    double distance;

    /**
     * Holds the data determined by Coral
     * @param name
     * @param heading
     * @param distance
     */
    public GameObject(String name, double heading, double distance) {
      this.name = name;
      this.heading = heading;
      this.distance = distance;
    }
  }

  NetworkTable table;
  int totalObjects;
  GameObject[] gameObjects;
  private String[] classes;
  private double[] boxes, box;

  public VisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("ML");
  }

  @Override
  public void periodic() {
    totalObjects = (int) table.getEntry("nb_objects").getNumber(0);
    gameObjects = new GameObject[totalObjects];
    classes = table.getEntry("object_classes").getStringArray(new String[totalObjects]);
    boxes = table.getEntry("boxes").getDoubleArray(new double[4 * totalObjects]);
    for (int i = 0; i < totalObjects; i += 4) {
      for (int j = 0; j < 4; j++) {
        box[j] = boxes[i + j];
      }
      gameObjects[i] = new GameObject(classes[i], heading, distance);
    }
  }

  private double getHeading(double[] box) {
    return 757.8125 / (Math.pow(Math.abs(box[2] - box[0]), -1.303));
  }

  private double getDistance(double[] box) {
    return (((box[0] + box[2]) / 2.0 - 160) / (Math.abs(box[2] - box[0]) / 19.5)) / 12.0;
  }
}
