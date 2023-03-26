// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDriveTimed extends CommandBase {
  /** Creates a new SwerveDriveTimed. */
  private Timer t;

  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;

  private double startTime;
  private double runTime;

  SwerveSubsystem swerve;
  public SwerveDriveTimed(SwerveSubsystem swerve, double xSpeed, double ySpeed, double rotSpeed, double runTime) {
    this.swerve = swerve;
    // Use addRequirements() here to declare subsystem dependencies.
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.runTime = runTime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    swerve.reset();
    swerve.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(xSpeed, ySpeed, rotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - startTime >= runTime;
  }
}
