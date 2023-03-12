// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

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
import frc.robot.commands.OuttakePiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ProfiledPIDArm;
import frc.robot.subsystems.ProfiledPIDElevator;
import frc.robot.subsystems.SwerveSubsystem;

public final class AutoRoutines {

  // WORK IN PROGRESS
  public static CommandBase chargingStation(SwerveSubsystem swerve, ProfiledPIDArm arm, ProfiledPIDElevator elevator) {
    return Commands.sequence(
      new ExtendArmElevatorAutoTest(arm, elevator, -0.3, 0.15),
      new DriveForwardDistance(swerve, 0.5).alongWith(arm.goToPosition(1.33))
    );
    
  }

  public static CommandBase LeftCubeConeAuto(SwerveSubsystem swerve, ProfiledPIDArm arm, ProfiledPIDElevator elevator, IntakeSubsystem intake, Limelight camera) {
    return new DoublePieceAutoFactory(swerve, arm, elevator, intake, camera, "Left_2Cube_P1", "Left_2Cube_P2", 5, 3);
  }

  public static CommandBase oneCubeAuto(ProfiledPIDArm arm, ProfiledPIDElevator elevator, IntakeSubsystem intake) {
    return Commands.sequence(
      new HomeElevator(elevator).alongWith(arm.goToPosition(1.33)),
      new ExtendArmElevatorAutoTest(arm, elevator, 0, 1.26),
      new WaitUntilCommand(() -> arm.atGoal() && elevator.atGoal()),
      new OuttakePiece(intake),
      new ExtendArmElevatorAutoTest(arm, elevator, 1.33, 0)
    );
  }

  public static CommandBase followTrajectory(SwerveSubsystem swerve, Trajectory trajectory) {

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

  public static Command baseSwerveCommand(PathPlannerTrajectory trajectory, SwerveSubsystem swerve, boolean isFirstPath) {
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

  

  private AutoRoutines() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  // public static Command Left_2Cube_P1(SwerveSubsystem swerve, ProfiledPIDArm arm, ProfiledPIDElevator elevator,
  //     IntakeSubsystem intake, Limelight camera) {
  //   return Commands.sequence(
  //     new startAuto(arm, elevator),
  //     new ExtendArmElevatorAutoTest(arm, elevator, AutoConstants.armPreset[5], AutoConstants.elevatorPreset[5]),
  //     new WaitUntilCommand(() -> arm.atGoal() && elevator.atGoal()),
  //     new OuttakePiece(intake)
  //   );
  // }
}
