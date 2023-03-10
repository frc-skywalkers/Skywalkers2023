// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ExtendArmElevatorAutoTest;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.Macros;
import frc.robot.commands.OuttakePiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ProfiledPIDArm;
import frc.robot.subsystems.ProfiledPIDElevator;
import frc.robot.subsystems.SwerveSubsystem;

public final class AutoRoutines {

  private final SwerveSubsystem swerve;
  private final ProfiledPIDElevator elevator;
  private final ProfiledPIDArm arm;
  private final IntakeSubsystem intake;
  private final Limelight limelight;

  private final Macros macros;


  public AutoRoutines(
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

    macros = new Macros(swerve, elevator, arm, intake, limelight);

  }

  // WORK IN PROGRESS
  public CommandBase chargingStation() {
    return Commands.sequence(
      macros.moveToPreset(0.15, -0.3),
      new DriveForwardDistance(swerve, 0.5).alongWith(arm.goToPosition(1.33)),
      new Balance(swerve)
    );
    
  }

  public CommandBase LeftCubeConeAuto() {
    return new DoublePieceAutoFactory(swerve, arm, elevator, intake, limelight, "Left_2Cube_P1", "Left_2Cube_P2", 5, 3);
  }

  public CommandBase oneCubeAuto() {
    return Commands.sequence(
      macros.home(),
      macros.cone3rdStage(),
      macros.outtake(),
      macros.stow()
    );
  }

  public CommandBase twoCubeAuto() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Left_2Cube_P1", 2, 3);
    PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("Left_2Cube_P2", 2, 3);
    
    return Commands.sequence(
      oneCubeAuto(),
      baseSwerveCommand(trajectory, true),
      macros.groundIntake(true),
      // Drive forward
      Commands.parallel(
        Commands.sequence(
          macros.stow(), 
          Commands.waitSeconds(2),
          macros.cube2ndStage()),
          baseSwerveCommand(trajectory2, false)),
      macros.outtake(),
      macros.stow()
    );
  }

  public CommandBase followTrajectory(Trajectory trajectory) {

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory, 
            swerve::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,        
            swerve::setModuleStatesClosedLoop,
            swerve);

    // 5. Add some init and wrap-up, and return everything
    return Commands.sequence(
      Commands.runOnce(
        () -> swerve.resetOdometry(trajectory.getInitialPose()), swerve),
      swerveControllerCommand,
      Commands.runOnce(
        () -> swerve.stopModules(), swerve)
    );
  }

  public Command baseSwerveCommand(PathPlannerTrajectory trajectory, boolean isFirstPath) {
    InstantCommand resetOdom = new InstantCommand(() -> {
      if(isFirstPath) {
        swerve.resetOdometry(trajectory.getInitialHolonomicPose());
        swerve.resetOdometry(trajectory.getInitialHolonomicPose());
      }
    });
    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
      trajectory, 
      swerve::getTrajectoryPose, 
      DriveConstants.kDriveKinematics, 
      new PIDController(3, 0, 0), 
      new PIDController(3, 0, 0), 
      new PIDController(3, 0, 0), 
      swerve::setModuleStatesClosedLoop, 
      swerve);
    return Commands.sequence(resetOdom, command);
  }

}
