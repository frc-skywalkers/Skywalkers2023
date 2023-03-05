// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ProfiledPIDArm;
import frc.robot.subsystems.ProfiledPIDElevator;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTesting extends SequentialCommandGroup {
  /** Creates a new AutoTesting. */
  public AutoTesting(ProfiledPIDElevator elevator, ProfiledPIDArm arm, SwerveSubsystem swerve, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new HomeElevator(elevator).alongWith(
      Commands.runOnce(() -> {
        arm.setGoal(1.33);
        arm.enable();
      }, arm)),

      new IntakePiece(intake),
      new DriveForwardDistance(swerve, 1));
  }
}
