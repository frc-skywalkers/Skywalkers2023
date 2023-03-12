// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;


import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExtendArmElevatorAutoTest;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.IntakePiece;
import frc.robot.commands.OuttakePiece;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ProfiledPIDArm;
import frc.robot.subsystems.ProfiledPIDElevator;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoublePieceAutoFactory extends SequentialCommandGroup {


  /** Creates a new Left_2Cube. */
  public DoublePieceAutoFactory(SwerveSubsystem swerve, ProfiledPIDArm arm, ProfiledPIDElevator elevator, IntakeSubsystem intake, Limelight camera, String trajPart1, String trajPart2, int scoreID1, int scoreID2) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // for this command we start from the left grid cube
    // we score an l3 cube, then we have to go to the left most game piece
    // we intake then move back to the start position

    SequentialCommandGroup scoreFirstCube = new SequentialCommandGroup(new startAuto(arm, elevator),
    new ExtendArmElevatorAutoTest(arm, elevator, AutoConstants.armPreset[scoreID1], AutoConstants.elevatorPreset[scoreID1]),
    new WaitUntilCommand(() -> arm.atGoal() && elevator.atGoal()),
    new OuttakePiece(intake).withTimeout(2));

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              .setKinematics(DriveConstants.kDriveKinematics);

  // Path path1 = Filesystem.getDeployDirectory().toPath().resolve("Left_2Cube_P1.wpilib.json");
  // Path path2 = Filesystem.getDeployDirectory().toPath().resolve("Left_2Cube_P2.wpilib.json");

  // Trajectory trajectory = new Trajectory();
  // Trajectory trajectory2 = new Trajectory();

  PathPlannerTrajectory trajectory = PathPlanner.loadPath("Left_2Cube_P1", 2, 3);
  PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("Left_2Cube_P2", 2, 3);

  // try {
  //   trajectory = TrajectoryUtil.fromPathweaverJson(path1);
  //   trajectory2 = TrajectoryUtil.fromPathweaverJson(path2);
  // } catch (IOException e) {
  //   e.printStackTrace();
  // }
// 2. Generate trajectory
// trajectory = TrajectoryGenerator.generateTrajectory(
//       new Pose2d(0, 0, new Rotation2d(0)),
//       List.of(
//               new Translation2d(-3, 0)),
//       new Pose2d(-3, 0, Rotation2d.fromDegrees(180)),
//       trajectoryConfig);
  // return Autos.followTrajectory(swerve, trajectory);


  // trajectory2 = TrajectoryGenerator.generateTrajectory(
  //   new Pose2d(-3, 0, new Rotation2d(180)),
  //   List.of(
  //           new Translation2d(0, 0)),
  //   new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
  //   trajectoryConfig);


    // PathPlannerTrajectory traj = PathPlanner.loadPath(trajPart1, new PathConstraints(4, 3));
    // PPSwerveControllerCommand drive1 = AutoRoutines.baseSwerveCommand(traj, swerve);




    ParallelCommandGroup driveToSecondCone = new ParallelCommandGroup(
      new ExtendArmElevatorAutoTest(arm, elevator, AutoConstants.armPreset[6], AutoConstants.elevatorPreset[6]), 
      new IntakePiece(intake),
      Autos.followTrajectory(swerve, trajectory)
    );

    // PathPlannerTrajectory traj2 = PathPlanner.loadPath(trajPart2, new PathConstraints(4, 3));
    // PPSwerveControllerCommand drive2 = AutoRoutines.baseSwerveCommand(traj2, swerve);

    // ParallelCommandGroup driveToConeGrid = new ParallelCommandGroup(
    //   new ExtendArmElevatorAutoTest(arm, elevator, AutoConstants.armPreset[scoreID2], AutoConstants.elevatorPreset[scoreID2]),
    //   Autos.followTrajectory(swerve, trajectory2)
    // );

    // addCommands(      
    //   scoreFirstCube,
    //   driveToSecondCone,
    //   driveToConeGrid,
    //   new OuttakePiece(intake).withTimeout(2),
    //   new ExtendArmElevatorAutoTest(arm, elevator, AutoConstants.armPreset[0], AutoConstants.elevatorPreset[0]),
    //   new HomeElevator(elevator)
    // );
    addCommands(
      AutoRoutines.baseSwerveCommand(trajectory, swerve, true),
      new WaitCommand(0.5),
      AutoRoutines.baseSwerveCommand(trajectory2, swerve, false)
    ); 
    // new FollowPathWithEvents(, null, null)
  }

  // zero then score first piece

  }  
