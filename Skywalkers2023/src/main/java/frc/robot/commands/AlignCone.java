// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//NEED TO CHECK AND RENAME

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

public class AlignCone extends CommandBase {

  private final PIDController xcontroller = new PIDController(8, 0, 0); //all in degrees
  private final PIDController ycontroller = new PIDController(8, 0, 0);
  private final PIDController rcontroller = new PIDController(0.15, 0, 0);

  private final double targetXDist;
  private final double targetYDist;
  private final double targetR; 

  double xspeed;
  double yspeed;
  double rspeed;
  double minxspeed;
  double minyspeed;

  SwerveSubsystem swerveSubsystem;
  Limelight camera;

  boolean atSetpoint;
  boolean ydistreached;
  boolean xdistreached;

  public AlignCone(SwerveSubsystem swerveSubsystem, Limelight camera, double targetXDist, double targetYDist, double targetR) { //meters, meters, degrees
    this.targetXDist = targetXDist; //positive forward
    this.targetYDist = targetYDist; //positive when robot to the right
    this.targetR = targetR;

    this.swerveSubsystem = swerveSubsystem;
    this.camera = camera;
    addRequirements(swerveSubsystem);
    addRequirements(camera);
  }

  @Override
  public void initialize() {
    xcontroller.setTolerance(LimelightConstants.xtolerance);
    ycontroller.setTolerance(LimelightConstants.ytolerance);
    rcontroller.setTolerance(LimelightConstants.rtolerance);
    ydistreached = false;
    xdistreached = false;

    /* 
    if (swerveSubsystem.getFieldOriented()){
      swerveSubsystem.toggleField();
    }
    */
  }

  @Override
  public void execute() {
    double currentYAngle = camera.getRTTX(); //-, limelight and swerve directions swapped, ref frame (robot to the right +)
    double currentXAngle = camera.getRTTY(); 
    //double currentR = camera.getRTTS(); //
    double currentR = swerveSubsystem.getHeading(); //

    double currentXdistance = (LimelightConstants.RTheight - LimelightConstants.cameraheight)/Math.tan(currentXAngle*Math.PI/180); //radians
    double currentYdistance = Math.tan(currentYAngle*Math.PI/180) * currentXdistance; //+

    //double targetXAngle = Math.atan((LimelightConstants.RTheight - LimelightConstants.cameraheight)/targetXDist) - LimelightConstants.mountingangle; //upwards angle
    //double targetYAngle = Math.atan(((targetYDist - LimelightConstants.limelightOffsetCenter)/targetXDist)); //+-?

    double robottargetY = targetYDist + LimelightConstants.limelightOffsetCenter;

    xspeed = 1 * MathUtil.clamp((xcontroller.calculate(currentXdistance, targetXDist)), -LimelightConstants.xclamp, LimelightConstants.xclamp);
    yspeed = 1 * MathUtil.clamp((ycontroller.calculate(currentYdistance, robottargetY)), -LimelightConstants.yclamp, LimelightConstants.yclamp); //-
    rspeed = 0;

    //rspeed = -0.5 * MathUtil.clamp((rcontroller.calculate(currentR, targetR)), -LimelightConstants.rclamp, LimelightConstants.rclamp);
    
    minxspeed = 0.5 * (xspeed/Math.abs(xspeed));
    minyspeed = 0.5 * (yspeed/Math.abs(yspeed));


    if (xcontroller.atSetpoint()){
      xspeed = -minxspeed;
      xdistreached = true;
    }
    if (ycontroller.atSetpoint()){
      yspeed = -minyspeed;
      ydistreached = true;
    }
    if (rcontroller.atSetpoint()){
      rspeed = 0;
    }

    if (Math.abs(xspeed) < 0.2){
      xspeed = -minxspeed;
    }
    if (Math.abs(yspeed) < 0.2){
      yspeed = -minyspeed;
    }
    if (Math.abs(rspeed) < 0.2){
      rspeed = 0;
    }
 
    //may have to add min speed
    /* 
    SmartDashboard.putNumber("rspeed", rspeed);
    SmartDashboard.putBoolean("ydistreached", ydistreached);
    SmartDashboard.putNumber("xspeed", xspeed);
    SmartDashboard.putNumber("yspeed", yspeed);

    SmartDashboard.putNumber("xerror", targetXAngle-currentXAngle);
    SmartDashboard.putNumber("yerror", targetYAngle-currentYAngle);
    */
    SmartDashboard.putNumber("xdistance", currentXdistance);
    SmartDashboard.putNumber("ydistance", currentYdistance);


    atSetpoint = (xcontroller.atSetpoint() && ycontroller.atSetpoint() && rcontroller.atSetpoint());

    swerveSubsystem.drive(0, yspeed+minyspeed, 0); //xspeed is forward? yspeed is sideways, rspeed is rotational?

    //+y = right?
    //+x = forward
  }

  @Override
  public void end(boolean interrupted) {
    /* 
    if (!swerveSubsystem.getFieldOriented()){
      swerveSubsystem.toggleField();
    }
    */
  }

  @Override
  public boolean isFinished() {
    return ((atSetpoint) || (xspeed == -minxspeed && yspeed == -minyspeed && rspeed == 0));
  }
}
