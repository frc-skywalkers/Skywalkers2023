// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ElevatorGoToPosition;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;


import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private static final Command auto = null;

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final XboxController driverJoystick = new XboxController(OIConstants.kDriverControllerPort);
  private final XboxController driverJoystick2 = new XboxController(OIConstants.kDriverControllerPort2);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings


    swerveSubsystem.setDefaultCommand(new SwerveJoystick(
      swerveSubsystem,
       driverJoystick));

    elevatorSubsystem.setDefaultCommand(Commands.runOnce(() -> elevatorSubsystem.setSpeed(driverJoystick2.getLeftY()), elevatorSubsystem));
//     armSubsystem.setDefaultCommand(Commands.run(() -> armSubsystem.setSpeed(0.5 * driverJoystick.getRightY()), armSubsystem));
    armSubsystem.setDefaultCommand(Commands.run(() -> armSubsystem.setSpeed(0.5 * driverJoystick2.getRightY()), armSubsystem));

//      elevatorSubsystem.setDefaultCommand(new ElevatorGoToPosition(elevatorSubsystem, 5, driverJoystick));



    configureButtonBindings();
  }

  /**
   * Use this method to defi ne your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
//     new JoystickButton(driverJoystick, Button.kX.value).whenPressed(() -> swerveSubsystem.zeroHeading());
    //new JoystickButton(driverJoystick)

    // use only this for testing purposes for now
    // if it works for all 4 swerve modules then you can use joystick
    // check the testMotors function in swerveSubsystem for more info
    
    new JoystickButton(driverJoystick, Button.kY.value).onTrue(Commands.runOnce(() -> swerveSubsystem.reset(), swerveSubsystem));
    new JoystickButton(driverJoystick, Button.kB.value).onTrue(Commands.runOnce(() -> swerveSubsystem.toggleField(), swerveSubsystem));
//     new JoystickButton(driverJoystick, Button.kX.value).onTrue(Commands.runOnce(() -> elevatorSubsystem.moveUp(), elevatorSubsystem));
//     new JoystickButton(driverJoystick, Button.kA.value).onTrue(Commands.runOnce(() -> elevatorSubsystem.moveDown(), elevatorSubsystem));
//     new JoystickButton(driverJoystick, Button.kB.value).onTrue(Commands.runOnce(() -> elevatorSubsystem.stop(), elevatorSubsystem));
//     new JoystickButton(driverJoystick, Button.kRightBumper.value).onTrue(Commands.runOnce(() -> armSubsystem.moveArmDown(), armSubsystem));
    
//     new JoystickButton(driverJoystick2, Button.kY.value).onTrue(Commands.runOnce(() -> elevatorSubsystem.stop(), elevatorSubsystem));
//     new JoystickButton(driverJoystick2, Button.kB.value).onTrue(Commands.runOnce(() -> armSubsystem.stop(), armSubsystem));
    new JoystickButton(driverJoystick2, Button.kX.value).onTrue(Commands.run(() -> intakeSubsystem.stopIntake(), intakeSubsystem));
    new JoystickButton(driverJoystick2, Button.kLeftBumper.value).onTrue(Commands.run(() -> intakeSubsystem.moveIn(), intakeSubsystem));
    new JoystickButton(driverJoystick2, Button.kRightBumper.value).onTrue(Commands.run(() -> intakeSubsystem.moveOut(), intakeSubsystem));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

      // 1. Create trajectory settings
      TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

// 2. Generate trajectory        
Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
                new Translation2d(2.5, 0),
                new Translation2d(2.5, -2.5),
                new Translation2d(0, -2.5)),
        new Pose2d(0, 0, Rotation2d.fromDegrees(180)),
        trajectoryConfig);

// 3. Define PID controllers for tracking trajectory
PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
thetaController.enableContinuousInput(-Math.PI, Math.PI);

// 4. Construct command to follow trajectory
SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory, 
        swerveSubsystem::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,        
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

// 5. Add some init and wrap-up, and return everything
return new SequentialCommandGroup(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> swerveSubsystem.stopModules()));
}
}