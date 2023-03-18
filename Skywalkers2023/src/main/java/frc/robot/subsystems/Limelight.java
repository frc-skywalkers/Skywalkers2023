// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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

  public int getId() {
    //targetID = limelightTable.getEntry("tid").getInteger(0);
    return (targetID);
  }

  public double[] getCamtoTarget() {
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (cameratotarget);
  }

  public double getTX(){
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (cameratotarget[0]);
  }

  public double getTY(){
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (cameratotarget[1]);
  }

  public double getTZ(){
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (cameratotarget[2]);
  }

  public double getRX(){
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (cameratotarget[3]);
  }

  public double getRY(){
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (cameratotarget[4]);
  }

  public double getRZ(){
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //x, y, z, in meters. roll, pitch, yaw in degrees  (translation + rotation)
    return (cameratotarget[5]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
