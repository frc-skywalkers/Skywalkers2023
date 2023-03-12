// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autos.DoublePieceAutoFactory;
import frc.robot.commands.Macros;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ProfiledPIDArm;
import frc.robot.subsystems.ProfiledPIDElevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;

public class RobotContainer {

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  public final ProfiledPIDElevator elevator = new ProfiledPIDElevator();
  public final ProfiledPIDArm arm = new ProfiledPIDArm();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final Limelight limelight = new Limelight();

  private final CommandXboxController driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController operatorJoystick = new CommandXboxController(OIConstants.kDriverControllerPort2);

  private final Macros macros = new Macros(swerve, elevator, arm, intake, limelight);

  public RobotContainer() {


    swerve.setDefaultCommand(new SwerveJoystick(swerve, driverJoystick));

    elevator.setDefaultCommand(Commands.run(() -> {
      double speed = -operatorJoystick.getLeftY() * ElevatorConstants.kMaxElevatorSpeed;
      elevator.setSpeed(speed);
    }, elevator).unless(elevator::isEnabled));

    
    arm.setDefaultCommand(Commands.run(() -> {
      double speed = -operatorJoystick.getRightY() * ArmConstants.kMaxArmSpeed;
      arm.setSpeed(speed);
    }, arm).unless(arm::isEnabled));

    configureButtonBindings();
  }


  private void configureButtonBindings() {

    driverJoystick.y().onTrue(Commands.runOnce(() -> swerve.reset(), swerve));
    driverJoystick.b().onTrue(Commands.runOnce(() -> swerve.toggleField(), swerve));
    // driverJoystick.x().onTrue(new MoveToTag(swerve, limelight, 1, 0, 0));

    // driverJoystick.a().onTrue(new Balance(swerve));
    driverJoystick.rightBumper().onTrue(Commands.runOnce(swerve::stopModules, swerve));

    // Right Trigger --> manual override
    operatorJoystick.rightTrigger().onTrue(
      Commands.runOnce(() -> {
        arm.stop();
        arm.disable();
        elevator.stop();
        elevator.disable();
      }, arm, elevator));

    // Start --> Home
    operatorJoystick.start().onTrue(macros.home());

    
    // POV Left --> First Stage / Ground intake height
    operatorJoystick.povLeft().onTrue(
      macros.groundIntake(false)
    );

    // POV Down --> Stowe
    operatorJoystick.povDown().onTrue(
      macros.stow()
    );

    // POV Up --> Substration intake height
    operatorJoystick.povUp().onTrue(
      macros.substationIntake(false)
    );

    // X --> Cone 2nd Stage
    operatorJoystick.x().onTrue(
      macros.cone2ndStage()
    );

    // Y --> Cone 3rd Stage
    operatorJoystick.y().onTrue(
      macros.cone3rdStage()
    );

    // A --> Cube 2nd Stage
    operatorJoystick.a().onTrue(
      macros.cube2ndStage()
    );

    // B --> Cube 3rd Stage
    operatorJoystick.b().onTrue(
      macros.cube3rdStage()
    );
    
    // Right Bumper --> Intake 
    operatorJoystick.rightBumper().onTrue(
      macros.intake()
    );

    // Left Bumper --> Outtake
    operatorJoystick.leftBumper().onTrue(
      macros.outtake()
    );

    // Back --> Manual Intake Stop
    operatorJoystick.back().onTrue(Commands.runOnce(() -> intake.stop(), intake));

    // driverJoystick.a().onTrue(Commands.run(() -> swerve.drive(1.00, 0.000, 0.000), swerve));

  }

  public Command getAutonomousCommand() {
    // return Autos.oneCubeAuto(arm, elevator, intake);
    PathPlannerTrajectory traj = PathPlanner.loadPath("Straight Path", new PathConstraints(2, 1.5));
    boolean isFirstPath = true;

    // PPSwerveControllerCommand autoDrive = baseSwerveCommand(traj);

  //   return new SequentialCommandGroup(
  //     new InstantCommand(() -> {
  //       // Reset odometry for the first path you run during auto
  //       if(isFirstPath){
  //           swerve.resetEncoders();
  //           // swerve.resetOdometry(traj.getInitialHolonomicPose());
  //       }
  //     }),
  //     new PPSwerveControllerCommand(
  //         traj, 
  //         swerve::getPose, // Pose supplier
  //         DriveConstants.kDriveKinematics, // SwerveDriveKinematics
  //         new PIDController(2, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
  //         new PIDController(2, 0, 0), // Y controller (usually the same values as X controller)
  //         new PIDController(0.5, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
  //         swerve::setModuleStatesClosedLoop, // Module states consumer
  //         true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  //         swerve // Requires this drive subsystem
  //     )

  // );

    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              .setKinematics(DriveConstants.kDriveKinematics);

// 2. Generate trajectory
Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
              new Translation2d(1, -1)),
      new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
      trajectoryConfig);
  // return Autos.followTrajectory(swerve, trajectory);
    return new DoublePieceAutoFactory(swerve, arm ,elevator, intake, limelight, "BS", "random", 1, 1);
  }


}
