// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.SwerveSubsystem;


public class TurnAngle extends CommandBase {
  /** Creates a new DriveForwardDistance. */
  private final PIDController rcontroller = new PIDController(0.15, 0, 0);

  private final double targetR; 

  SwerveSubsystem swerveSubsystem;

  boolean atSetpoint;

  double rspeed;
  double currentR;

  public TurnAngle(SwerveSubsystem swerveSubsystem, double targetR) { //meters, meters, degrees
    this.targetR = targetR;

    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    rcontroller.setTolerance(LimelightConstants.rtolerance);
  }

  @Override
  public void execute() {
    currentR = swerveSubsystem.getHeading(); //

    rspeed = 0.5 * MathUtil.clamp((rcontroller.calculate(currentR, targetR)), -LimelightConstants.rclamp, LimelightConstants.rclamp);
    
    rspeed += 0.5 * Math.signum(rspeed);

    if (rcontroller.atSetpoint()){
      rspeed = 0;
    }

    
    SmartDashboard.putNumber("rspeed", rspeed);


    atSetpoint = rcontroller.atSetpoint();

    swerveSubsystem.drive(0, 0, rspeed);
  }

  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return ((Math.abs(targetR - currentR) < LimelightConstants.rtolerance) || (rspeed == 0));
  }
}