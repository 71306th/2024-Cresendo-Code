// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  private BooleanArrayPublisher tled;
  private DoubleSubscriber tv;
  private DoubleSubscriber tx;
  private DoubleSubscriber ty;
  private DoubleSubscriber ta;
  private DoubleSubscriber tid;
  private DoubleArraySubscriber coordinate;

  private double valid;
  private double x;
  private double y;
  private double area;
  private double id;
  private double[] coordinateArr;
  private boolean[] ledArr;

  NetworkTable table;
  
  public Vision() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getDoubleTopic("tv").subscribe(0.0);
    tx = table.getDoubleTopic("tx").subscribe(0.0);
    ty = table.getDoubleTopic("ty").subscribe(0.0);
    ta = table.getDoubleTopic("ta").subscribe(0.0);
    tid = table.getDoubleTopic("tid").subscribe(0.0);
    coordinate = table.getDoubleArrayTopic("targetpose_cameraspace").subscribe(new double[]{});
    ledArr = new boolean[]{false, false, false, false};
    tled.set(ledArr);
  }
  
  @Override
  public void periodic() {

    /* getting values */
    valid = tv.get();
    x = tx.get();
    y = ty.get();
    area = ta.get();
    id = tid.get();
    coordinateArr = coordinate.get();

    SmartDashboard.putNumber("LimelightVaild", valid);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightID", id);
    SmartDashboard.putNumberArray("LimelightToTarget", coordinateArr);
  }

  public double getID() {
    return id;
  }

  public Pose3d TagToBotCenter() {
    Translation3d translation3d = new Translation3d(
    coordinateArr[0]+Constants.Vision.CameraToBotCenter.getX(), 
    coordinateArr[1]+Constants.Vision.CameraToBotCenter.getY(), 
    coordinateArr[2]+Constants.Vision.CameraToBotCenter.getZ());

    Rotation3d camera3d = new Rotation3d(coordinateArr[3], coordinateArr[4], coordinateArr[5]);
    Rotation3d rotation3d = camera3d.plus(Constants.Vision.CameraToBotCenter.getRotation());

    return new Pose3d(translation3d, rotation3d);
  }

  public void setLEDMode(int command){
    switch(command){
      case 0:
        ledArr[0] = true; ledArr[1] = false; ledArr[2] = false; ledArr[3] = false;
        break;
      case 1:
        ledArr[0] = false; ledArr[1] = true; ledArr[2] = false; ledArr[3] = false;
        break;
      case 2:
        ledArr[0] = false; ledArr[1] = false; ledArr[2] = true; ledArr[3] = false;
        break;
      case 3:
        ledArr[0] = false; ledArr[1] = false; ledArr[2] = false; ledArr[3] = true;
        break;
    }
    tled.set(ledArr);
    tled = table.getBooleanArrayTopic("ledMode").publish();
  }
}