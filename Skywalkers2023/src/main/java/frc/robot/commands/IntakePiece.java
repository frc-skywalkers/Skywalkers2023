// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePiece extends CommandBase {
  private final IntakeSubsystem intake;

  private boolean finished = false;

  private Timer speedUp = new Timer();

  private int stage;

  //private CommandXboxController controller;
  /** Creates a new IntakeMotor. */
  public IntakePiece(IntakeSubsystem rIntake) {
    //controller = Controller;
    intake = rIntake;
    addRequirements(rIntake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    stage = 0;
    finished = false;
    intake.moveIn();
    speedUp.reset();
    speedUp.start();
    intake.stop = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(stage == 0) {
      if(intake.speedUp()) { // waits for power to go up
        stage = 1;
        speedUp.stop();
      } else {
        SmartDashboard.putNumber("Trying speedup", speedUp.get()); //keeps trying for set amount time, if it doesn't work, it stops
          if(speedUp.get() > IntakeConstants.kSpeedUpFailTime) {
            stage = -1;
            intake.stop();
            speedUp.stop();
            finished = true;
          } else {
            intake.moveIn();
          }
      }
    } else { //starts checking for game pieces once its speed up
      if(intake.objectHeld()) { // waits for spike when object is intaked
        intake.stop();
        finished = true;
        stage = -1;
      } else {
        intake.moveIn();
      }
    }
    SmartDashboard.putNumber("Intake Step", stage);
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
