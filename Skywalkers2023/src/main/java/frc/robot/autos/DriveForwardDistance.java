// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;


public class DriveForwardDistance extends CommandBase {
  /** Creates a new DriveForwardDistance. */
  private final SwerveSubsystem swerve;
  private double distance;
  private double start;
  public DriveForwardDistance(SwerveSubsystem swerve, double distance) {
    this.swerve = swerve;
    this.distance = distance;
    start = swerve.getPose().getX();
    addRequirements(swerve);
    SmartDashboard.putNumber("Start X", start);
    SmartDashboard.putNumber("Current X", swerve.getPose().getX());
    SmartDashboard.putNumber("Distance Travelled", swerve.getPose().getX() - start);
    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Start X", start);
    SmartDashboard.putNumber("Current X", swerve.getPose().getX());
    SmartDashboard.putNumber("Distance Travelled", swerve.getPose().getX() - start);
    swerve.drive(1, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerve.getPose().getX() - start >= distance;
  }
}
