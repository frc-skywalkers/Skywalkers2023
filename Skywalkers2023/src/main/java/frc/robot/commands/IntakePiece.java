// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Dashboard;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePiece extends CommandBase {
  private final IntakeSubsystem intake;

  private boolean finished = false;

  private int stage;

  private int piece;

  //private CommandXboxController controller;
  /** Creates a new IntakeMotor. */
  public IntakePiece(IntakeSubsystem rIntake) {
    intake = rIntake;
    
    addRequirements(rIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public IntakePiece(IntakeSubsystem rIntake, int rPiece) {
    intake = rIntake;
    piece = rPiece;
    //piece = intake.getPiece();
    addRequirements(rIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    piece = intake.getPiece();
    stage = 0;
    finished = false;
    intake.moveIn(piece);
    intake.stop = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
      //intake.currentPiece = piece;
      intake.holdObject();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished || intake.stop;
  }
}
