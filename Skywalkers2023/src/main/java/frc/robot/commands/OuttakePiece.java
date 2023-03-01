// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakePiece extends CommandBase {
  private final IntakeSubsystem intake;

  private int stage = 0;

  private boolean finished = false;

  private Timer outFail = new Timer();
  /** Creates a new IntakeMotor. */
  public OuttakePiece(IntakeSubsystem rIntake) {
    intake = rIntake;
    addRequirements(rIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.moveOut();
    finished = false;
    stage = 0;
    outFail.reset();
    outFail.start();
    intake.stop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(stage == 0) {
      if(!intake.objectOut()) { //waits for the spike when the piece is still stuck
        stage = 1;
      } else {
        intake.moveOut();
      }
    } else {
      if(intake.objectOut()) { // now it waits for spike to lower, i.e. object out 
        stage = -1;
        intake.stop();
        outFail.stop();
        finished = true;
      } else { //sometimes it fails when the cone is stuck, this to prevent damage to the motors
        SmartDashboard.putNumber("Trying outtake", outFail.get());
        if(outFail.get() > IntakeConstants.kOutFailTime) {
          stage = -1;
          intake.stop();
          outFail.stop();
          finished = true;
        } else {
          intake.moveOut();
        }
      }
    }
    SmartDashboard.putNumber("Outtake Step", stage);
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
