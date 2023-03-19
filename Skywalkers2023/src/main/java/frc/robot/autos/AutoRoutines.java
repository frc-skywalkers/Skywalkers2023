// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.commands.IntakePiece;
import frc.robot.commands.Macros;
import frc.robot.commands.OuttakePiece;
import frc.robot.commands.SwerveDriveTimed;
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

    PathPlannerTrajectory trajectory = PathPlanner.loadPath("charge_station_P1", 2.5, 3);
    // PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("charge_station_P2", 2.5, 3);

    return Commands.sequence(
      cube3rdAuto(),
      baseSwerveCommand(trajectory, true),
      new Balance(swerve)
    );
    
  }

  public CommandBase coneChargingStation() {

    PathPlannerTrajectory trajectory = PathPlanner.loadPath("charge_station_P1", 2.5, 3);
    // PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("charge_station_P2", 2.5, 3);

    return Commands.sequence(
      cone2ndAuto(),
      baseSwerveCommand(trajectory, true),
      new Balance(swerve)
    );
    
  }

  public CommandBase cube3rdMobilityRight() {

    PathPlannerTrajectory trajectory = PathPlanner.loadPath("1_Cube_Mobility_Right", 2.5, 3);

    return Commands.sequence(
      cube3rdAuto(),
      baseSwerveCommand(trajectory, true)
    );
    
  }

  public CommandBase cube3rdMobilityLeft() {

    PathPlannerTrajectory trajectory = PathPlanner.loadPath("1_Cube_Mobility_Left", 2.5, 3);

    return Commands.sequence(
      cube3rdAuto(),
      baseSwerveCommand(trajectory, true)
    );
    
  }

  public CommandBase LeftCubeConeAuto() {
    return new DoublePieceAutoFactory(swerve, arm, elevator, intake, limelight, "Left_2Cube_P1", "Left_2Cube_P2", 5, 3);
  }

  public CommandBase cube3rdAuto() {
    return Commands.sequence(
      macros.home(),
      macros.cube3rdStage(),
      macros.outtake(),
      macros.stow()
    );
  }

  public CommandBase cone2ndAuto() {
    return cube2ndAuto();
  } 

  public CommandBase cube2ndAuto() {
    return Commands.sequence(
      macros.home(),
      macros.cube2ndStage(),
      macros.outtake(),
      macros.stow()
    );
  }

  public CommandBase Cone3rdBalance() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("charge_station_P1", 2.5, 3);

    return Commands.sequence(
      cone3rdAuto(),
      baseSwerveCommand(trajectory, true),
      new Balance(swerve)
    );
  }

  public CommandBase Cube2ndBalance() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("charge_station_P1", 2.5, 3);

    return Commands.sequence(
      cube2ndAuto(),
      baseSwerveCommand(trajectory, true),
      new Balance(swerve)
    );
  }
  
  public CommandBase cone3rdAuto() {
    return Commands.sequence(
      macros.home(),
      macros.cone3rdStage(),
      macros.outtake(),
      macros.stow()
    );
  }

  public CommandBase twoCubeAuto() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Left_2Cube_P1", 2.0, 2.75);
    PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("Left_2Cube_P2", 2.5, 2.75);
    
    return Commands.sequence(
      macros.home(),
      macros.cube3rdStage(),
      macros.outtake(),
      macros.stow(),
      Commands.parallel(
        baseSwerveCommand(trajectory, true), 
        Commands.waitSeconds(1.5).andThen(macros.groundIntake(true))),
      Commands.parallel(
        macros.stow(),
        Commands.waitSeconds(1).andThen(baseSwerveCommand(trajectory2, false))),
      macros.cone3rdStage(),
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
        swerve.reset();
        swerve.reset();
        swerve.resetOdometry(trajectory.getInitialHolonomicPose());
        swerve.resetOdometry(trajectory.getInitialHolonomicPose());
      }
    });
    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
      trajectory, 
      swerve::getTrajectoryPose, 
      DriveConstants.kDriveKinematics, 
      new PIDController(5, 0, 0), 
      new PIDController(5, 0, 0), 
      new PIDController(3, 0, 0), 
      swerve::setModuleStatesClosedLoop, 
      swerve);
    return Commands.sequence(resetOdom, command);
  }

  public CommandBase DiagnosticTest() {
    return Commands.sequence(      
      new HomeElevator(elevator), 
      new SwerveDriveTimed(swerve, 1.00, 0, 0, 2.00), 
      new SwerveDriveTimed(swerve, 0, 1, 0, 2.000),
      new SwerveDriveTimed(swerve, 0.00, 0.00, 1.00, 2.000),
      macros.stow(),
      macros.cube3rdStage(),
      macros.cone3rdStage(),
      macros.stow(),
      new IntakePiece(intake),
      new OuttakePiece(intake)
    );
  }

}
