// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//NEED TO CHECK AND RENAME

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard.Limelight.Debugging;
import frc.robot.Constants.LimelightConstants;



public class Limelight extends SubsystemBase {

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

    /*
     * (TY, TZ, TX, RY, RZ, RX)
     * left/right
     * up/down
     * front/back
     * rotation about left/right (y) axis, pitch
     * rotation about up/down (z) axis, yaw
     * rotation about front/back (x) axis, roll
     */

     //tx: left/right, ty: up/down, tz: forward/backward
  }

  public Transform3d CamtoTarget() { //order maybe incorrect??
    var t = getCamtoTarget();
    /*
     * (x, y, z, roll, pitch, yaw) 
     * forward/backward, along length of field
     * left/right, along width of field
     * up/down
     * rotation about left/right axis
     * rotation about forward/backward axis
     * rotation about up/down (z) axis, yaw
     * 
     * field orientation (x and y) switched, ordering switched
     */
    return new Transform3d(new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)), new Pose3d(t[2], t[0], t[1], new Rotation3d(t[5], t[3], t[4])));
  }

  public static final Pose3d[] blueGameAprilTags = { //copied, check all
     new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)),
    //new Pose3d(15.51, 1.07, 0.46, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(15.51, 2.74, 0.46, new Rotation3d(0, 0, Math.PI)), //field layout
    new Pose3d(15.51, 4.42, 0.46, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(16.18, 6.75, 0.69, new Rotation3d(0, 0, Math.PI)),
    new Pose3d(0.36, 6.75, 0.69, new Rotation3d(0, 0, 0)),
    new Pose3d(1.03, 4.42, 0.46, new Rotation3d(0, 0, 0)),
    new Pose3d(1.03, 4.42, 0.46, new Rotation3d(0, 0, 0)),
    // new Pose3d(1.03, 2.74, 0.46, new Rotation3d(0, 0, 0)),
    new Pose3d(1.03, 1.07, 0.46, new Rotation3d(0, 0, 0))
  };

  public Pose3d campose() {
    var targetPose = blueGameAprilTags[getId()];
    return targetPose.transformBy(CamtoTarget().inverse()); //target to camera
  }

  public double getTY(){ //left right, y controller
    limelightTable.getEntry("pipeline").setNumber(0);
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); //z(up/down), y(left/right), x(forward/backward), rx?, ry(left/right), rz?
    return (cameratotarget[0]);
  }

  public double getTZ(){ //up down, not used
    limelightTable.getEntry("pipeline").setNumber(0);
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); 
    return (cameratotarget[1]);
  }

  public double getTX(){  //front back, x controller
    limelightTable.getEntry("pipeline").setNumber(0);
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); 
    return (cameratotarget[2]);
  }

  public double getRY(){ 
    limelightTable.getEntry("pipeline").setNumber(0);
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); 
    return (cameratotarget[3]);
  }

  public double getRZ(){ //rotation used
    limelightTable.getEntry("pipeline").setNumber(0);
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); 
    return (cameratotarget[4]);
  }

  public double getRX(){
    limelightTable.getEntry("pipeline").setNumber(0);
    cameratotarget = limelightTable.getEntry("targetpose_cameraspace").getDoubleArray(new double[6]); 
    return (cameratotarget[5]);
  }

  //retroreflective tape pipeline

  public double getRTTX(){ //left/right
    limelightTable.getEntry("pipeline").setNumber(1);
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    double x = tx.getDouble(0.0);
    return x;
  }

  public double getRTTY(){ //up/down
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

  public double getObjectTX(){
    limelightTable.getEntry("pipeline").setNumber(2);
    NetworkTableEntry tx = limelightTable.getEntry("tx");
    double x = tx.getDouble(0.0);
    return x;
  }

  public double getObjectTY(){
    limelightTable.getEntry("pipeline").setNumber(2);
    NetworkTableEntry ty = limelightTable.getEntry("ty");
    double y = ty.getDouble(0.0);
    return y;
  }

  @Override
  public void periodic() {

    int pipeline = limelightTable.getEntry("pipeline").getNumber(0).intValue();

    if (pipeline == 1) {
      Debugging.putNumber("RTTX", getRTTX());
      Debugging.putNumber("RTTY", getRTTY());
      Debugging.putNumber("RTTA", getRTTA());
      double currentXdistance = (LimelightConstants.RTheight - LimelightConstants.cameraheight)/Math.tan(getRTTY()*Math.PI/180); //radians
      double currentYdistance = Math.tan(getRTTX()*Math.PI/180) * currentXdistance; //+

      Debugging.putNumber("xdist", currentXdistance);
      Debugging.putNumber("ydist", currentYdistance);
      Debugging.putNumber("Printing pipeline", 1);
    } else if (pipeline == 0) {
      Debugging.putNumber("TAGID", getId());
      Debugging.putNumber("Printing pipeline", 0);
    }
    

    
    
    
  }
}
