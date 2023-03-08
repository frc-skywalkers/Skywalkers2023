// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//NEED TO CHECK AND RENAME

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.Dashboard;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

public class MoveToTag extends CommandBase {
  /** Creates a new MoveToTag. */

  private final PIDController xcontroller = new PIDController(LimelightConstants.kPx, LimelightConstants.kIx, LimelightConstants.kDx);
  private final PIDController ycontroller = new PIDController(LimelightConstants.kPy, LimelightConstants.kIy, LimelightConstants.kDy);
  private final PIDController rcontroller = new PIDController(LimelightConstants.kPr, LimelightConstants.kIr, LimelightConstants.kDr);

  private final double targetXDist;
  private final double targetYDist;
  private final double targetR;
  
  double xspeed;
  double rspeed;
  double yspeed;

  SwerveSubsystem swerveSubsystem;
  Limelight camera;

  boolean ydistreached;
  boolean atSetpoint;

  public MoveToTag(SwerveSubsystem swerveSubsystem, Limelight camera, double targetXDist, double targetYDist, double targetR) {
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
    ydistreached = false;

    /* 
    if (swerveSubsystem.getFieldOriented()){
      swerveSubsystem.toggleField();
    }
    */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentXDist = camera.getTX(); 
    double currentYDist = camera.getTY();
    double currentR = camera.getRZ();

    xspeed = -1 * MathUtil.clamp((xcontroller.calculate(currentXDist, targetXDist)), -LimelightConstants.xclamp, LimelightConstants.xclamp);
    yspeed = -1 * MathUtil.clamp((ycontroller.calculate(currentYDist, targetYDist - LimelightConstants.limelightOffsetCenter)), -LimelightConstants.yclamp, LimelightConstants.yclamp);
    rspeed = -0.5 * MathUtil.clamp((rcontroller.calculate(currentR, targetR)), -LimelightConstants.rclamp, LimelightConstants.rclamp);
    
    Dashboard.Limelight.Debugging.putNumber("rspeed", rspeed);
    Dashboard.Limelight.Debugging.putNumber("xspeed", xspeed);
    Dashboard.Limelight.Debugging.putNumber("yspeed", yspeed);

    Dashboard.Limelight.Debugging.putBoolean("ydistreached", ydistreached);

    swerveSubsystem.drive(xspeed, yspeed, rspeed); //Have to recheck for swerve subsystem

    //stopping individually since command only ends with all 3
    if (xcontroller.atSetpoint()){
      xspeed = 0;
    }
    if (ycontroller.atSetpoint()){
      ydistreached = true;
      yspeed = 0;
    }
    if (rcontroller.atSetpoint()){
      rspeed = 0;
    }

    //stopping abrupt movement at end
    if (Math.abs(xspeed) < 0.02){
      xspeed = 0;
    }
    if (Math.abs(yspeed) < 0.02){
      yspeed = 0;
    }
    if (Math.abs(rspeed) < 0.02){
      rspeed = 0;
    }

    atSetpoint = (xcontroller.atSetpoint() && ycontroller.atSetpoint() && rcontroller.atSetpoint());

    //+y = right
    //+x = forward
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /* 
    if (!swerveSubsystem.getFieldOriented()){
      swerveSubsystem.toggleField();
    }
    */
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ((atSetpoint) || (xspeed == 0 && yspeed == 0 && rspeed == 0));
  }
}
