// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  // private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  // private final ArmSubsystem arm = new ArmSubsystem();
  // private final IntakeSubsystem intake = new IntakeSubsystem();

  private final CommandXboxController driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController operatorJoystick = new CommandXboxController(OIConstants.kDriverControllerPort2);


  public RobotContainer() {

    // swerve.setDefaultCommand(new SwerveJoystick(swerve, driverJoystick));

    elevator.setDefaultCommand(Commands.runOnce(() -> {
      double speed = -operatorJoystick.getLeftY() * 0.4;
      SmartDashboard.putNumber("Elevator Speed", speed);
      elevator.setSpeed(speed);
    }, elevator));

    // arm.setDefaultCommand(Commands.run(() -> arm.setSpeed(0.5 * operatorJoystick.getRightY()), arm));

    configureButtonBindings();
  }


  private void configureButtonBindings() {


    operatorJoystick.a().onTrue(new HomeElevator(elevator));
    operatorJoystick.x().onTrue(Commands.runOnce(elevator::stop));
    operatorJoystick.y().onTrue(Commands.runOnce(elevator::resetEncoders));

    
    // driverJoystick.y().onTrue(Commands.runOnce(() -> swerve.reset(), swerve));
    // driverJoystick.b().onTrue(Commands.runOnce(() -> swerve.toggleField(), swerve));

    // operatorJoystick.x().onTrue(Commands.run(() -> intake.stopIntake(), intake));
    // operatorJoystick.leftBumper().onTrue(Commands.run(() -> intake.moveIn(), intake));
    // operatorJoystick.rightBumper().onTrue(Commands.run(() -> intake.moveOut(), intake));


  }

  public Command getAutonomousCommand() {
    return Commands.none();

        
}
}