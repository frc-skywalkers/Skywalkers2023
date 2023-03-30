// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCharacterization extends CommandBase {
  /** Creates a new ArmCharacterization. */
  ArmSubsystem arm;
  CommandXboxController joystick;
  double speed = 0.00;
  double volts = 0.00;
  public ArmCharacterization(ArmSubsystem arm, CommandXboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.joystick = joystick;
   SmartDashboard.putNumber("feed forward volts", 0.000);
   SmartDashboard.putNumber("arm characterization speed", 0.000);
    SmartDashboard.putNumber("arm passed speed", speed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick.a().getAsBoolean()) {
      SmartDashboard.putNumber("A Pressed", 0);
      speed = 1.00;
    } 
    if(joystick.b().getAsBoolean()) {
      SmartDashboard.putNumber("B Pressed", 0);
      speed = -1.00;
    }
    if(joystick.x().getAsBoolean()) {
      SmartDashboard.putNumber("X Pressed", 0);
      speed = 0.00;
    }

    if(joystick.getLeftTriggerAxis() > 0.500) {
      volts += 0.5000;
    }
    if(joystick.getRightTriggerAxis() > 0.5000) {
      volts -= 0.50000;
    }
    // volts = SmartDashboard.getNumber("feed forward volts", volts);
    double finSpeed = volts * speed;
    arm.setVoltage(finSpeed);
    SmartDashboard.putNumber("arm characterization speed", arm.getVelocity());
    SmartDashboard.putNumber("arm passed speed", finSpeed);
    SmartDashboard.putNumber("feed forward volts", volts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return joystick.y().getAsBoolean();
  }
}
