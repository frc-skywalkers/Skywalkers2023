// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveToObject extends CommandBase {
  /** Creates a new MoveToObject. */

  private final PIDController xcontroller = new PIDController(LimelightConstants.kPx, LimelightConstants.kIx, LimelightConstants.kDx);
  private final PIDController ycontroller = new PIDController(LimelightConstants.kPy, LimelightConstants.kIy, LimelightConstants.kDy);
  private final PIDController rcontroller = new PIDController(LimelightConstants.kPr, LimelightConstants.kIr, LimelightConstants.kDr);

  private final double targetXDist;
  private final double targetYDist;
  private final double targetR;

  double currXDist;
  double currYDist;

  double xspeed;
  double yspeed;

  SwerveSubsystem swerveSubsystem;
  Limelight camera;

  public MoveToObject(SwerveSubsystem swerveSubsystem, Limelight camera, double targetXDist, double targetYDist, double targetR){
    this.targetXDist = targetXDist;
    this.targetYDist = targetYDist;
    this.targetR = targetR;

    this.swerveSubsystem = swerveSubsystem;
    this.camera = camera;
    
    addRequirements(swerveSubsystem);
    addRequirements(camera);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xcontroller.setTolerance(LimelightConstants.xtolerance);
    ycontroller.setTolerance(LimelightConstants.ytolerance);
    rcontroller.setTolerance(LimelightConstants.rtolerance);

    swerveSubsystem.toggleField();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currYAngle = camera.getObjectTY() + LimelightConstants.mountingangle;
    double currXAngle = camera.getObjectTX();

    currXDist = (LimelightConstants.cameraheight - LimelightConstants.objectHeight) * Math.tan(currYAngle * (Math.PI/180)); //rad
    currYDist = currXDist*Math.tan(currXAngle * (Math.PI/180)); //rad

    double robotTargetY = targetYDist + LimelightConstants.limelightOffsetCenter;

    xspeed = -1 * MathUtil.clamp((xcontroller.calculate(currXDist, targetXDist)), -LimelightConstants.xclamp, LimelightConstants.xclamp);
    yspeed = 1 * MathUtil.clamp((ycontroller.calculate(currYDist, robotTargetY)), -LimelightConstants.yclamp, LimelightConstants.yclamp);


    //stopping individually since command only ends with all 3
    if (xcontroller.atSetpoint()){
      xspeed = 0;
    }
    if (ycontroller.atSetpoint()){
      yspeed = 0;
    }
    
    //stopping abrupt movement at end
    if (Math.abs(xspeed) < 0.02){
      xspeed = 0;
    }
    if (Math.abs(yspeed) < 0.02){
      yspeed = 0;
    }

    swerveSubsystem.drive(xspeed, yspeed, 0);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
    swerveSubsystem.toggleField();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((Math.abs(targetXDist - currXDist) < LimelightConstants.xtolerance) ||
      (Math.abs(targetYDist - currYDist) < LimelightConstants.ytolerance) ||
      (xspeed == 0 && yspeed == 0));
  }
}
