// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Piece;

public class OuttakePiece extends CommandBase {
  private final IntakeSubsystem intake;

  private boolean finished = false;

  private Piece piece;

  /** Creates a new IntakeMotor. */
  public OuttakePiece(IntakeSubsystem rIntake) {
    intake = rIntake;
    
    addRequirements(rIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public OuttakePiece(IntakeSubsystem rIntake, Piece kPiece) {
    intake = rIntake;
    intake.setCurrentPiece(kPiece);
    addRequirements(rIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }
   
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    piece = intake.getCurrentPiece();
    if (piece == Piece.NONE) {
      piece = intake.getDesiredPiece();
    }
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
    // if (!interrupted) {
      // intake.setCurrentPiece(Piece.NONE);
    // }
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished || intake.stop;
  }
}
