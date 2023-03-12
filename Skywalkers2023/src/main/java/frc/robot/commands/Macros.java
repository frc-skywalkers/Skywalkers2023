// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Presets;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ProfiledPIDArm;
import frc.robot.subsystems.ProfiledPIDElevator;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class Macros {

  private final SwerveSubsystem swerve;
  private final ProfiledPIDElevator elevator;
  private final ProfiledPIDArm arm;
  private final IntakeSubsystem intake;
  private final Limelight limelight;


  public Macros(
      SwerveSubsystem swerve, 
      ProfiledPIDElevator elevator, 
      ProfiledPIDArm arm, 
      IntakeSubsystem intake, 
      Limelight limelight) {
    
    this.swerve = swerve;
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    this.limelight = limelight;

  }

  public CommandBase home() {
    return Commands.parallel(
      new HomeElevator(elevator),
      arm.goToPosition(Presets.STOW_PRESET.kArmPos)
    );
  }

  public CommandBase moveToPreset(double elevatorPos, double armPos) {
    return Commands.parallel(
      arm.goToPosition(armPos),
      elevator.goToPosition(elevatorPos)
    );
  }

  public CommandBase stow() {
    return moveToPreset(
      Presets.STOW_PRESET.kElevatorPos, 
      Presets.STOW_PRESET.kArmPos);
  }

  public CommandBase groundIntake(boolean intakeOn) {
    return moveToPreset(
      Presets.GROUND_INTAKE_PRESET.kElevatorPos, 
      Presets.GROUND_INTAKE_PRESET.kArmPos)
      .andThen(new IntakePiece(intake).unless(() -> !intakeOn));
  }

  public CommandBase substationIntake(boolean intakeOn) {
    return moveToPreset(
      Presets.SUBSTATION_INTAKE_PRESET.kElevatorPos, 
      Presets.SUBSTATION_INTAKE_PRESET.kArmPos)
      .andThen(new IntakePiece(intake).unless(() -> !intakeOn));
  }

  public CommandBase cube2ndStage() {
    return moveToPreset(
      Presets.CUBE_2ND_STAGE_PRESET.kElevatorPos, 
      Presets.CUBE_2ND_STAGE_PRESET.kArmPos);
  }

  public CommandBase cube3rdStage() {
    return moveToPreset(
      Presets.CUBE_3RD_STAGE_PRESET.kElevatorPos, 
      Presets.CUBE_3RD_STAGE_PRESET.kArmPos);
  }

  public CommandBase cone2ndStage() {
    return moveToPreset(
      Presets.CONE_2ND_STAGE_PRESET.kElevatorPos, 
      Presets.CONE_2ND_STAGE_PRESET.kArmPos);
  }

  public CommandBase cone3rdStage() {
    return moveToPreset(
      Presets.CONE_3RD_STAGE_PRESET.kElevatorPos, 
      Presets.CONE_3RD_STAGE_PRESET.kArmPos);
  }

  public CommandBase intake() {
    return new IntakePiece(intake);
  }

  public CommandBase outtake() {
    return new OuttakePiece(intake).withTimeout(2);
  }



}
