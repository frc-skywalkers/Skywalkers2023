// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakePiece extends CommandBase {
  private final IntakeSubsystem intake;

  private boolean finished = false;

  private final int piece;

  /** Creates a new IntakeMotor. */
  public OuttakePiece(IntakeSubsystem rIntake) {
    intake = rIntake;
    piece = intake.getCurrentPiece();
    addRequirements(rIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public OuttakePiece(IntakeSubsystem rIntake, int kPiece) {
    intake = rIntake;
    piece = kPiece;
    addRequirements(rIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }
   
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.moveOut(piece);
    finished = false;
    intake.stop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.intakeEmpty()) {
      intake.stop();
      finished = true;
    } else {
      intake.moveOut(piece);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished || intake.stop;
  }
}
