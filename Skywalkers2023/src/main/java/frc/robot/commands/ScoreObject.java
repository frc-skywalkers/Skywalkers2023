// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ProfiledPIDArm;
import frc.robot.subsystems.ProfiledPIDElevator;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreObject extends SequentialCommandGroup {
  /** Creates a new ScoreObject. */
  public ScoreObject(SwerveSubsystem swerve, ProfiledPIDArm arm, ProfiledPIDElevator elev, Limelight camera, IntakeSubsystem intake, int scoringID) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    double armPreset = AutoConstants.armPreset[scoringID];
    double elevPreset = AutoConstants.elevatorPreset[scoringID];

    // later potentially align with swerve + limelight, for now stick with this

    addCommands(new ExtendArmElevatorAutoTest(arm, elev, armPreset, elevPreset), new OuttakePiece(intake));
  }
}
