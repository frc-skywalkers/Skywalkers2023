// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.HomeElevator;
import frc.robot.subsystems.ProfiledPIDArm;
import frc.robot.subsystems.ProfiledPIDElevator;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class startAuto extends ParallelCommandGroup {
  /** Creates a new startAuto. */

  private final ProfiledPIDArm arm;
  private final ProfiledPIDElevator elev;

  public startAuto(ProfiledPIDArm arm, ProfiledPIDElevator elev) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.arm = arm;
    this.elev = elev;
    addCommands(new HomeElevator(this.elev), this.arm.goToPosition(AutoConstants.armPreset[0]));
  }
}
