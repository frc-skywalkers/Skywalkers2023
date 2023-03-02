// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
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

  SwerveSubsystem swerveSubsystem;
  Limelight camera;

  public MoveToTag(SwerveSubsystem swerveSubsystem, Limelight camera, double targetXDist, double targetYDist, double targetR) {
    this.targetXDist = targetXDist;
    this.targetYDist = targetYDist;
    this.targetR = targetR;

    this.swerveSubsystem = swerveSubsystem;
    this.camera = camera;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xcontroller.setTolerance(LimelightConstants.xtolerance);
    ycontroller.setTolerance(LimelightConstants.ytolerance);
    rcontroller.setTolerance(LimelightConstants.rtolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentXDist = camera.getTZ();
    double currentYDist = camera.getTX();
    double currentR = camera.getRY();

    double xspeed = -1 * MathUtil.clamp((xcontroller.calculate(currentXDist, targetXDist)), -LimelightConstants.xclamp, LimelightConstants.xclamp);
    double yspeed = -1 * MathUtil.clamp((ycontroller.calculate(currentYDist, targetYDist)), -LimelightConstants.yclamp, LimelightConstants.yclamp);
    double rspeed = -0.5 * MathUtil.clamp((rcontroller.calculate(currentR, targetR)), -LimelightConstants.rclamp, LimelightConstants.rclamp);

    swerveSubsystem.drive(xspeed, yspeed, rspeed); //Have to recheck for swerve subsystem

    //stopping individually since command only ends with all 3
    if (xcontroller.atSetpoint()){
      xspeed = 0;
    }
    if (ycontroller.atSetpoint()){
      yspeed = 0;
    }
    if (ycontroller.atSetpoint()){
      rspeed = 0;
    }

    //stopping abrupt movement at end
    if (xspeed < 0.02){
      xspeed = 0;
    }
    if (yspeed < 0.02){
      yspeed = 0;
    }
    if (rspeed < 0.02){
      rspeed = 0;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (xcontroller.atSetpoint() && ycontroller.atSetpoint() && rcontroller.atSetpoint());
  }
}
