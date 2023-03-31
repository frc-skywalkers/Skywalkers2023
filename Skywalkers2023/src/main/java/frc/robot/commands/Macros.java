// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.LimelightConstants.*;
import frc.robot.Preset;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Presets;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Lightstrip;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ProfiledPIDElevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Mode;

/** Add your docs here. */
public class Macros {

  private final SwerveSubsystem swerve;
  private final ProfiledPIDElevator elevator;
  private final ArmSubsystem arm;
  private final IntakeSubsystem intake;
  private final Limelight limelight;


  public Macros(
      SwerveSubsystem swerve, 
      ProfiledPIDElevator elevator, 
      ArmSubsystem arm, 
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

  public CommandBase moveToPreset(Preset preset) {
    return Commands.parallel(
      arm.goToPosition(preset.kArmPos),
      elevator.goToPosition(preset.kElevatorPos)
    );
  }


  public CommandBase stow() {
    return moveToPreset(
      Presets.STOW_PRESET.kElevatorPos, 
      Presets.STOW_PRESET.kArmPos);
  }

  public CommandBase setCubeMode() {
    return Commands.runOnce(() -> {
      intake.setMode(Mode.CUBE);
    });
  }

  public CommandBase setConeMode() {
    return Commands.runOnce(() -> {
      intake.setMode(Mode.CONE);
    });
  }

  public CommandBase groundIntake(boolean intakeOn, Mode m) {
    return Commands.sequence(
      Commands.runOnce(() -> intake.setMode(m)),
      groundIntake(),
      new IntakePiece(intake).unless(() -> !intakeOn)
    );
  }

  public CommandBase substationIntake(boolean intakeOn, Mode m) {
    return Commands.sequence(
      Commands.runOnce(() -> intake.setMode(m)),
      substationIntake(),
      new IntakePiece(intake).unless(() -> !intakeOn)
    );
  }

  public CommandBase groundIntake() {
    return Commands.runOnce(() -> {
      Mode m = intake.getMode();
      if (m == Mode.CONE) {
        CommandScheduler.getInstance().schedule(moveToPreset(Presets.GROUND_INTAKE_CONE_PRESET));
      } else if (m == Mode.CUBE) {
        CommandScheduler.getInstance().schedule(moveToPreset(Presets.GROUND_INTAKE_CUBE_PRESET));
      } else {
        System.out.println("GROUND INTAKE ERROR");
      }
    });
  }

  public CommandBase groundIntake(Mode m) {
    return Commands.runOnce(() -> {
      if (m == Mode.CONE) {
        CommandScheduler.getInstance().schedule(moveToPreset(Presets.GROUND_INTAKE_CONE_PRESET));
      } else if (m == Mode.CUBE) {
        CommandScheduler.getInstance().schedule(moveToPreset(Presets.GROUND_INTAKE_CUBE_PRESET));
      } else {
        System.out.println("GROUND INTAKE ERROR");
      }
    }
    );
  }

  public CommandBase substationIntake() {
      return Commands.runOnce(() -> {
        Mode m = intake.getMode();
        if (m == Mode.CONE) {
          CommandScheduler.getInstance().schedule(moveToPreset(Presets.SUBSTATION_INTAKE_CONE_PRESET));
        } else if (m == Mode.CUBE) {
          CommandScheduler.getInstance().schedule(moveToPreset(Presets.SUBSTATION_INTAKE_CUBE_PRESET));
        } else {
          System.out.println("SUBSTATION INTAKE ERROR");
        }
      }
    );
  }

  public CommandBase singleSubstationIntake() {
    return Commands.runOnce(() -> {
      Mode m = intake.getMode();
      if (m == Mode.CONE) {
        CommandScheduler.getInstance().schedule(moveToPreset(Presets.SINGLE_SUBSTATION_CONE));
      } else if (m == Mode.CUBE) {
        CommandScheduler.getInstance().schedule(moveToPreset(Presets.SINGLE_SUBSTATION_CUBE));
      } else {
        System.out.println("SUBSTATION INTAKE ERROR");
      }
    }
  );
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
    return new IntakePiece(intake).andThen(Commands.run(() -> {
      intake.holdObject();
    }, intake).until(() -> {return intake.getSpeed() > (IntakeConstants.kHoldSpeed / 2);}));
  }

  public CommandBase intake(Lightstrip lightstrip) {
    return new IntakePiece(intake, lightstrip).andThen(Commands.run(() -> {
      intake.holdObject();
    }, intake).until(() -> {return intake.getSpeed() > (IntakeConstants.kHoldSpeed / 2);}));
  }

  public CommandBase intake(Mode m) {
    return new IntakePiece(intake, m).andThen(Commands.run(() -> {
      intake.holdObject();
    }, intake).until(() -> {return intake.getSpeed() > (IntakeConstants.kHoldSpeed / 2);}));
  }

  public CommandBase outtake() {
    return new OuttakePiece(intake).withTimeout(1);
  }

  public CommandBase outtake(Mode m) {
    return new OuttakePiece(intake, m).withTimeout(1);
  }

  public CommandBase alignCone2ndStage() {
    return new AlignCone(swerve, limelight, SecondStageConeConstants.targetXMeters, SecondStageConeConstants.targetYMeters, SecondStageConeConstants.targetRDeg);
  }

  public CommandBase scoreCone2ndStage() {
    return Commands.sequence(
      new TurnAngle(swerve, 0),
      alignCone2ndStage(),
      cone2ndStage(),
      moveToPreset(Presets.CONE_2ND_STAGE_PRESET.kElevatorPos, 0),
      outtake(),
      stow()
    );
  }

  public CommandBase scoreCone3rdStage() {
    return Commands.sequence(
      new TurnAngle(swerve, 0),
      alignCone2ndStage(),
      cone3rdStage(),
      outtake(),
      stow()
    );
  }

  public CommandBase general2ndStage() {
    return Commands.runOnce(() -> {
        Mode m = intake.getMode();
        System.out.println("INTAKE MODE 2: " + m);
        if (m == Mode.CONE) {
          CommandScheduler.getInstance().schedule(cone2ndStage());
        } else if (m == Mode.CUBE) {
          CommandScheduler.getInstance().schedule(cube2ndStage());
        } else {
          System.out.println("GENERAL 2ND STAGE ERROR");
          System.out.println("NO PIECE INSIDE INTAKE");
        }
      }
    );
  }

  public CommandBase general3rdStage() {
    return Commands.runOnce(() -> {
        Mode m = intake.getMode();
        System.out.println("INTAKE MODE 3: " + m);
        if (m == Mode.CONE) {
          CommandScheduler.getInstance().schedule(cone3rdStage());
        } else if (m == Mode.CUBE) {
          CommandScheduler.getInstance().schedule(cube3rdStage());
        } else {
          System.out.println("GENERAL 3RD STAGE ERROR");
          System.out.println("NO PIECE INSIDE INTAKE");
        }
      }
    );
  }

}
