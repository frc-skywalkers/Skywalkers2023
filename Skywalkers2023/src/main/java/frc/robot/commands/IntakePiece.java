// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Dashboard;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Piece;

public class IntakePiece extends CommandBase {
  private final IntakeSubsystem intake;

  private boolean finished = false;

  private int stage;

  private Piece piece;

  public IntakePiece(IntakeSubsystem rIntake) {
    intake = rIntake;
    addRequirements(rIntake);
  }

  public IntakePiece(IntakeSubsystem rIntake, Piece rPiece) {
    intake = rIntake;
    intake.setDesiredPiece(rPiece);
    addRequirements(rIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    stage = 0;
    finished = false;
    intake.moveIn(piece);
    intake.stop = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    piece = intake.getDesiredPiece();
    if(stage == 0) {
      if(intake.intakeEmpty()) { // waits for power to go up
        stage = 1;
      } else {
        intake.moveIn(piece);
      }
    } else { //starts checking for game pieces once its speed up
      if(intake.pieceHeld()) { // waits for spike when object is intaked
        intake.stop();
        finished = true;
        stage = -1;
      } else {
        intake.moveIn(piece);
      }
    }
    Dashboard.Intake.Debugging.putNumber("Intake Step", stage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted) {
      intake.setCurrentPiece(intake.getDesiredPiece());
      intake.holdObject();
    } else {
      intake.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished || intake.stop;
  }
}
