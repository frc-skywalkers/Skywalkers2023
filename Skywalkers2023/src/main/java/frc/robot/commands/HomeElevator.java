// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ProfiledPIDElevator;

public class HomeElevator extends CommandBase {
  /** Creates a new HomeElevator. */

  private final ProfiledPIDElevator elevator;

  public HomeElevator(ProfiledPIDElevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.disableSoftLimits();
    SmartDashboard.putBoolean("Zeroed", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setSpeed(ElevatorConstants.kHomingSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    elevator.resetEncoders();
    elevator.isZeroed = true;
    elevator.enableSoftLimits();
    SmartDashboard.putBoolean("Zeroed", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return elevator.getCurrent() > ElevatorConstants.kCurrentThreshold;
  }
}
