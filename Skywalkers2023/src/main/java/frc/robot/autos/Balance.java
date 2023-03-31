// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class Balance extends CommandBase {
  /** Creates a new Balance. */

  //275, 97.5

  SwerveSubsystem swerve;
  PIDController controller = new PIDController(0.05, 0, 0);

  public Balance(SwerveSubsystem swerve) {
    this.swerve = swerve;
    SmartDashboard.putNumber("balance speed ", 0.00);
    SmartDashboard.putNumber("balance roll", swerve.getRoll());
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(0);
    swerve.toggleField();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = controller.calculate(swerve.getRoll());
    SmartDashboard.putNumber("balance speed ", xSpeed);
    SmartDashboard.putNumber("balance roll", swerve.getRoll());
    xSpeed = -MathUtil.clamp(xSpeed, -1, 1);
    swerve.drive(xSpeed, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
    swerve.toggleField();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
