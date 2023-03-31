// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Dashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Presets;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.IntakePiece;
import frc.robot.commands.Macros;
import frc.robot.commands.OuttakePiece;
import frc.robot.commands.SwerveDriveTimed;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Lightstrip;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ProfiledPIDElevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Mode;

public final class AutoRoutines {

  private final SwerveSubsystem swerve;
  private final ProfiledPIDElevator elevator;
  private final ArmSubsystem arm;
  private final IntakeSubsystem intake;
  private final Limelight limelight;
  private final Lightstrip lightstrip;

  private final Macros macros;


  public AutoRoutines(
      SwerveSubsystem swerve, 
      ProfiledPIDElevator elevator, 
      ArmSubsystem arm, 
      IntakeSubsystem intake, 
      Limelight limelight,
      Lightstrip lightstrip) {
    
    this.swerve = swerve;
    this.elevator = elevator;
    this.arm = arm;
    this.intake = intake;
    this.limelight = limelight;
    this.lightstrip = lightstrip;


    macros = new Macros(swerve, elevator, arm, intake, limelight);

  }

  public CommandBase debugIntakePart() {
    return Commands.sequence(
      macros.home(),
      macros.setCubeMode(),
      Commands.runOnce(() -> SmartDashboard.putString("Before Ground Intake", "yes")),
      macros.cubeGroundIntake(),
      Commands.runOnce(() -> SmartDashboard.putString("After Ground Intake", "yes")),
      macros.intake(),
      Commands.runOnce(() -> SmartDashboard.putString("After Intake On", "yes"))
    );
  }

  public CommandBase debugOdometryReset(Pose2d testPose) {
    return Commands.runOnce(() -> {
      Pose2d currentPose = swerve.getPose();
      SmartDashboard.putNumber("Before Reset X", currentPose.getX());
      SmartDashboard.putNumber("Before Reset Y", currentPose.getY());
      SmartDashboard.putNumber("Before Resert R", currentPose.getRotation().getDegrees());
      swerve.resetOdometry(testPose);
      Pose2d resetPose = swerve.getPose();
      SmartDashboard.putNumber("After Reset X", resetPose.getX());
      SmartDashboard.putNumber("After Reset Y", resetPose.getY());
      SmartDashboard.putNumber("After Resert R", resetPose.getRotation().getDegrees());
    }, swerve);
  }

  // WORK IN PROGRESS
  public CommandBase chargingStation() {

    PathPlannerTrajectory trajectory = PathPlanner.loadPath("charge_station_P1", 2.5, 3);
    // PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("charge_station_P2", 2.5, 3);

    return Commands.sequence(
      cube3rdAuto(),
      Commands.runOnce(() -> SmartDashboard.putString("before move back", "yes")),
      baseSwerveCommand(trajectory, true),
      Commands.runOnce(() -> SmartDashboard.putString("after move back", "yes")),
      new Balance(swerve)
    );
    
  }

  public CommandBase leftConeCubeAuto() {
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("Left_Cone_Cube_Auto", 1, 1.5);

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("intakeDown", macros.setCubeMode().andThen(macros.cubeGroundIntake()).andThen(macros.intake()));
    eventMap.put("stow", macros.stow());
    eventMap.put("prepareScore", macros.cube3rdStage());

    FollowPathWithEvents grabConeAndPrepareToScore = new FollowPathWithEvents(
      baseSwerveCommand(trajectory, true), 
      trajectory.getMarkers(), 
      eventMap);

    return Commands.sequence(
      cone3rdAuto(),
      grabConeAndPrepareToScore,
      macros.outtake(),
      macros.stow()
      // Commands.runOnce(() -> swerve.reset(swerve.getHeading() + 180)).andThen(Commands.runOnce(() -> Dashboard.Auto.Debugging.putString("Reset", "RESET!"))),
      // Commands.runOnce(() -> swerve.reset(swerve.getHeading() + 180))
      
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

  public CommandBase cube3rdAuto() {
    return Commands.sequence(
      // Commands.runOnce(() -> swerve.setHeading(180), swerve),
      macros.home(),
      macros.setCubeMode(),
      macros.cube3rdStage(),
      macros.outtake(),
      macros.stow()
    );
  }

  public CommandBase cone2ndAuto() {
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setHeading(180), swerve),
      macros.home(),
      macros.setConeMode(),
      macros.cone2ndStage(),
      macros.outtake(),
      macros.stow()
    );
  } 

  public CommandBase cube2ndAuto() {
    return Commands.sequence(
      Commands.runOnce(() -> swerve.setHeading(180), swerve),
      macros.home(),
      macros.setCubeMode(),
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
      Commands.runOnce(() -> swerve.setHeading(180), swerve),
      macros.home(),
      macros.setConeMode(),
      elevator.goToPosition(Presets.CONE_3RD_STAGE_PRESET.kElevatorPos),
      arm.goToPosition(Presets.CONE_3RD_STAGE_PRESET.kArmPos),
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
        Commands.waitSeconds(1.5).andThen(macros.groundIntake(true, Mode.CUBE))),
      Commands.parallel(
        macros.stow(),
        Commands.waitSeconds(1).andThen(baseSwerveCommand(trajectory2, false))),
      macros.cone3rdStage(),
      macros.outtake(),
      macros.stow()
    );
  }

  public Command baseSwerveCommand(PathPlannerTrajectory trajectory, boolean isFirstPath) {
    InstantCommand resetOdom = new InstantCommand(() -> {
      if(isFirstPath) {
        swerve.resetOdometry(trajectory.getInitialHolonomicPose());
      }
    }, swerve);

    PPSwerveControllerCommand command = new PPSwerveControllerCommand(
      trajectory, 
      swerve::getPose, 
      DriveConstants.kDriveKinematics, 
      new PIDController(6.5, 0, 0), 
      new PIDController(6.5, 0, 0), 
      new PIDController(3, 0, 0), 
      swerve::setModuleStatesClosedLoop, 
      swerve);

    Timer timer = new Timer();
    SmartDashboard.putNumber("Desired X", 0);
          SmartDashboard.putNumber("Desired Y", 0);
          SmartDashboard.putNumber("Desired R", 0);
          SmartDashboard.putNumber("Error X", 0);
          SmartDashboard.putNumber("Error Y", 0);
          SmartDashboard.putNumber("Error R", 0);

    return Commands.sequence(
      resetOdom.andThen(Commands.runOnce(
        () -> {
          timer.reset();
          timer.start();
        })), 
      command.raceWith(
        Commands.run(() -> {
          double t = timer.get();
          PathPlannerState state = (PathPlannerState) trajectory.sample(t);
          SmartDashboard.putNumber("Desired X", state.poseMeters.getX());
          SmartDashboard.putNumber("Desired Y", state.poseMeters.getY());
          SmartDashboard.putNumber("Desired R", state.poseMeters.getRotation().getRadians());
          SmartDashboard.putNumber("Error X", state.poseMeters.getX() - swerve.getPose().getX());
          SmartDashboard.putNumber("Error Y", state.poseMeters.getY() - swerve.getPose().getY());
          SmartDashboard.putNumber("Error R", state.poseMeters.getRotation().getRadians() - swerve.getPose().getRotation().getRadians());
        })
      ), 
      Commands.runOnce(() -> SmartDashboard.putString("Path Ended", "yes")));
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
      new IntakePiece(intake, Mode.CUBE),
      new OuttakePiece(intake)
    );
  }

}
