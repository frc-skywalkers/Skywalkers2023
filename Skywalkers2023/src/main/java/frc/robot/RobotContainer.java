// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Balance;
import frc.robot.commands.DriveForwardDistance;
import frc.robot.commands.ExtendArmElevatorAutoTest;
import frc.robot.commands.HomeElevator;
import frc.robot.commands.IntakePiece;
import frc.robot.commands.OuttakePiece;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.TurnAngle;
import frc.robot.commands.MoveToTag;
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
    operatorJoystick.start().onTrue(new HomeElevator(elevator).alongWith(
      arm.goToPosition(1.33)));

    
    // POV Left --> First Stage / Ground intake height
    operatorJoystick.povLeft().onTrue(
      new ExtendArmElevatorAutoTest(arm, elevator, -0.19, 0.10)
    );

    // POV Down --> Stowe
    operatorJoystick.povDown().onTrue(
      new ExtendArmElevatorAutoTest(arm, elevator, 1.33, 0)
    );

    // POV Up --> Substration intake height
    operatorJoystick.povUp().onTrue(
      new ExtendArmElevatorAutoTest(arm, elevator, 0, 1.13)
    );

    // X --> Cone 2nd Stage
    operatorJoystick.x().onTrue(
      new ExtendArmElevatorAutoTest(arm, elevator, 0.8, 0.72)
    );

    // Y --> Cone 3rd Stage
    operatorJoystick.y().onTrue(
      new ExtendArmElevatorAutoTest(arm, elevator, 0.47, 1.38)
    );

    // A --> Cube 2nd Stage
    operatorJoystick.a().onTrue(
      new ExtendArmElevatorAutoTest(arm, elevator, 0, 0.85)
    );

    // B --> Cube 3rd Stage
    operatorJoystick.b().onTrue(
      new ExtendArmElevatorAutoTest(arm, elevator, 0, 1.26)
    );
    
    // Right Bumper --> Intake 
    operatorJoystick.rightBumper().onTrue(new IntakePiece(intake));

    // Left Bumper --> Outtake
    operatorJoystick.leftBumper().onTrue(new OuttakePiece(intake).withTimeout(2));

    // Back --> Manual Intake Stop
    operatorJoystick.back().onTrue(Commands.runOnce(() -> intake.stop(), intake));

    // driverJoystick.a().onTrue(Commands.run(() -> swerve.drive(1.00, 0.000, 0.000), swerve));

  }

  public Command getAutonomousCommand() {
    return Autos.oneCubeAuto(arm, elevator, intake);
    // PathPlannerTrajectory examplePath = PathPlanner.loadPath("Simple Path", new PathConstraints(4, 3));
    // boolean isFirstPath = true;
    // return new SequentialCommandGroup(
    //     new InstantCommand(() -> {
    //       // Reset odometry for the first path you run during auto
    //       if(isFirstPath){
    //           this.resetOdometry(traj.getInitialHolonomicPose());
    //       }
    //     }),
    //     new PPSwerveControllerCommand(
    //         traj, 
    //         this::getPose, // Pose supplier
    //         this.kinematics, // SwerveDriveKinematics
    //         new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //         new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
    //         new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
    //         this::setModuleStates, // Module states consumer
    //         true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    //         this // Requires this drive subsystem
    //     )
    // );

  }
}
