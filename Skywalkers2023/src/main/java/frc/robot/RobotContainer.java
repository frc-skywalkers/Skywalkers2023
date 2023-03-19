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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autos.AutoRoutines;
import frc.robot.autos.DoublePieceAutoFactory;
import frc.robot.autos.DriveForwardDistance;
import frc.robot.commands.Balance;
import frc.robot.commands.Macros;
import frc.robot.commands.MoveToTag;
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
  private final AutoRoutines autoRoutines = new AutoRoutines(swerve, elevator, arm, intake, limelight);

  SendableChooser<Command> m_Chooser = new SendableChooser<>();

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

    m_Chooser.setDefaultOption("3rd Stage Cube Balance", autoRoutines.chargingStation());
    m_Chooser.addOption("3rd Stage Cone Balance", autoRoutines.Cone3rdBalance());
    m_Chooser.addOption("2nd Stage Cone Balance", autoRoutines.coneChargingStation());
    m_Chooser.addOption("2nd Stage Cube Balance", autoRoutines.Cube2ndBalance());

    m_Chooser.addOption("3rd Stage Cube", autoRoutines.cube3rdAuto());
    m_Chooser.addOption("3rd Stage Cone", autoRoutines.cone3rdAuto());
    m_Chooser.addOption("2nd Stage Cone", autoRoutines.cone2ndAuto());
    m_Chooser.addOption("2nd Stage Cube", autoRoutines.cube2ndAuto());

    m_Chooser.addOption("2 Cube Auto", autoRoutines.twoCubeAuto());


    SmartDashboard.putData(m_Chooser);

    configureButtonBindings();
  }


  private void configureButtonBindings() {

    driverJoystick.y().onTrue(Commands.runOnce(() -> swerve.reset(), swerve));
    driverJoystick.b().onTrue(Commands.runOnce(() -> swerve.toggleField(), swerve));
    driverJoystick.x().onTrue(new MoveToTag(swerve, limelight, 0.79, 0, 0));

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
    return m_Chooser.getSelected();
  }


}
