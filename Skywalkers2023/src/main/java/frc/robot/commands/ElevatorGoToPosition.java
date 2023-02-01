// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorGoToPosition extends CommandBase {
  /** Creates a new ElevatorGoToPosition. */

  private final ElevatorSubsystem elevator;

  private final double goal;

  private static double kDt = 0.02;

  private final XboxController joystick;

  private final TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(1, 1.5);
  // private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  // private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  private final ProfiledPIDController m_controller = new ProfiledPIDController(ElevatorConstants.kPElevator, ElevatorConstants.kIElevator, ElevatorConstants.kDElevator, m_constraints, kDt);

  private final ElevatorFeedforward m_ElevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.kSElevator, ElevatorConstants.kGElevator, ElevatorConstants.kVElevator, ElevatorConstants.kAElevator);

  public ElevatorGoToPosition(ElevatorSubsystem elevator, double goal, XboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(elevator);

    this.elevator = elevator;
    this.goal = goal;
    this.joystick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // potentially change goal here if needed from joystick input
    final double elevPIDOutput = m_controller.calculate(elevator.getPosition(), goal);

    final double elevFeedforward = m_ElevatorFeedforward.calculate(m_controller.getSetpoint().velocity);

    elevator.setVoltage(elevPIDOutput + elevFeedforward);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    return;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return joystick.getAButton();
  }
}
