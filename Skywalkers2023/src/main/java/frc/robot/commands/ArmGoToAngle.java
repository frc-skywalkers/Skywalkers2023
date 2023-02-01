// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmGoToAngle extends CommandBase {

  private final ArmSubsystem arm;

  private final double goal;
  
  private static double kDt = 0.02;
  
  private final XboxController joystick;

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(1.5, 1);

  private final ProfiledPIDController m_controller = new ProfiledPIDController(ArmConstants.kPArm, ArmConstants.kIArm, ArmConstants.kDArm, m_constraints, kDt);

  private final ArmFeedforward m_ArmFeedforward = new ArmFeedforward(ArmConstants.kSArm, ArmConstants.kGArm, ArmConstants.kVArm, ArmConstants.kAArm);

  /** Creates a new ArmGoToAngle. */
  public ArmGoToAngle(ArmSubsystem arm, double goal, XboxController joystick) {

    addRequirements(arm);

    this.arm = arm;
    this.goal = goal;
    this.joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // potentially change goal here if needed from joystick input

    final double armPIDOutput = m_controller.calculate(arm.getPosition(), goal);

    final double armFeedforward = m_ArmFeedforward.calculate(m_controller.getSetpoint().position, m_controller.getSetpoint().velocity);

    arm.setVoltage(armPIDOutput + armFeedforward);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return joystick.getRightBumper();
  }
}
