// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Dashboard;
import frc.robot.subsystems.SwerveSubsystem;


public class TurnAngle extends CommandBase {
  /** Creates a new DriveForwardDistance. */
  private final SwerveSubsystem swerve;
  private double target;
  private double start;
  public TurnAngle(SwerveSubsystem swerve, double target) {
    this.swerve = swerve;
    this.target = target;
    start = swerve.getHeading();
    addRequirements(swerve);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Dashboard.Swerve.Debugging.putNumber("Angle Turned", swerve.getHeading() - start);
    swerve.drive(0, 0, 1.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerve.getHeading() - start >= target;
  }
}
