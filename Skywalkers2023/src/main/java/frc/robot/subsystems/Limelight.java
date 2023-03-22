// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//NEED TO CHECK AND RENAME

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  int targetID;
  double[] cameratotarget = new double[6];

  public Limelight() {
    //CameraServer.startAutomaticCapture(0);

    CameraServer.startAutomaticCapture(1); //LL2

    UsbCamera LL3 = CameraServer.startAutomaticCapture(0);
    LL3.setBrightness(50); //idk
    LL3.setExposureManual(15);
  }

  //fiducial markers pipeline

  public int getId() {
    limelightTable.getEntry("pipeline").setNumber(0);
    //targetID = limelightTable.getEntry("tid").getInteger(0);
    return (targetID);
  }

  public double[] getCamtoTarget() {
    limelightTable.getEntry("pipeline").setNumber(0);
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (cameratotarget);
  }

  public Transform3d CamtoTarget() { //order maybe incorrect??
    var t = getCamtoTarget();
    return new Transform3d(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), new Pose3d(t[0], t[1], t[2], new Rotation3d(t[3], t[4], t[5])));
  }

  public static final Pose3d[] blueGameAprilTags = { //copied, check all
    // new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
    new Pose3d(15.51, 1.07, 0.46, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(15.51, 2.74, 0.46, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(15.51, 4.42, 0.46, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(16.18, 6.75, 0.69, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(0.36, 6.75, 0.69, new Rotation3d(0, 0, 0)),
    new Pose3d(1.03, 4.42, 0.46, new Rotation3d(0, 0, 0)),
    new Pose3d(1.03, 4.42, 0.46, new Rotation3d(0, 0, 0)),
    // new Pose3d(1.03, 2.74, 0.46, new Rotation3d(0, 0, 0)),
    new Pose3d(1.03, 1.07, 0.46, new Rotation3d(0, 0, 0))
  };

  public Pose3d robotpose() {
    var targetPose = blueGameAprilTags[getId()];
    return targetPose.transformBy(CamtoTarget().inverse());
  }

  public double getTX(){ //front/back, used for ycontroller
    limelightTable.getEntry("pipeline").setNumber(0);
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (cameratotarget[0]);
  }

  public double getTY(){ //up/down, not used
    limelightTable.getEntry("pipeline").setNumber(0);
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (cameratotarget[1]);
  }

  public double getTZ(){ //left/right, used for xcontroller
    limelightTable.getEntry("pipeline").setNumber(0);
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (cameratotarget[2]);
  }

  public double getRX(){ //not used
    limelightTable.getEntry("pipeline").setNumber(0);
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (cameratotarget[3]);
  }

  public double getRY(){ //left/right, used for rcontroller
    limelightTable.getEntry("pipeline").setNumber(0);
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (cameratotarget[4]);
  }

  public double getRZ(){ //not used
    limelightTable.getEntry("pipeline").setNumber(0);
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (cameratotarget[5]);
  }

  //retroreflective tape pipeline

  public double getRTTX(){
    limelightTable.getEntry("pipeline").setNumber(1);
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    double x = tx.getDouble(0.0);
    return x;
  }

  public double getRTTY(){
    limelightTable.getEntry("pipeline").setNumber(1);
    NetworkTableEntry ty = limelightTable.getEntry("ty");
    double y = ty.getDouble(0.0);
    return y;
  }

  public double getRTTA(){
    limelightTable.getEntry("pipeline").setNumber(1);
    NetworkTableEntry ta = limelightTable.getEntry("ta");
    double a = ta.getDouble(0.0);
    return a;
  }

  public double getRTTS(){
    limelightTable.getEntry("pipeline").setNumber(1);
    NetworkTableEntry ts = limelightTable.getEntry("ts");
    double s = ts.getDouble(0.0);
    return s;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("RTTX", getRTTX());
    SmartDashboard.putNumber("RTTY", getRTTY());
    SmartDashboard.putNumber("RTTA", getRTTA());
    SmartDashboard.putNumber("TAGID", getId());
  }
}
