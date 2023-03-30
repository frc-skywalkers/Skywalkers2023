// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DashbaordConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.NewArmConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.autos.AutoRoutines;
import frc.robot.commands.ArmCharacterization;
import frc.robot.commands.IntakePiece;
import frc.robot.commands.Macros;
import frc.robot.commands.OuttakePiece;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.TurnAngle;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ProfiledPIDElevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Limelight;

public class RobotContainer {

  private final SwerveSubsystem swerve = new SwerveSubsystem();
  public final ProfiledPIDElevator elevator = new ProfiledPIDElevator();
  public final ArmSubsystem arm = new ArmSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final Limelight limelight = new Limelight();

  private final CommandXboxController driverJoystick = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController operatorJoystick = new CommandXboxController(OIConstants.kDriverControllerPort2);

  private final Macros macros = new Macros(swerve, elevator, arm, intake, limelight);
  private final AutoRoutines autoRoutines = new AutoRoutines(swerve, elevator, arm, intake, limelight);

  SendableChooser<Command> m_Chooser = new SendableChooser<>();

  public RobotContainer() {
    startDashboard();

    swerve.setDefaultCommand(new SwerveJoystick(swerve, driverJoystick));

    elevator.setDefaultCommand(Commands.run(() -> {
      double speed = -operatorJoystick.getLeftY() * ElevatorConstants.kMaxElevatorSpeed;
      elevator.setSpeed(speed);
    }, elevator).unless(elevator::isEnabled));

    arm.setDefaultCommand(Commands.run(() -> {
      double speed = -operatorJoystick.getRightY() * NewArmConstants.kMaxArmSpeed;
      speed = Math.abs(speed) > OIConstants.kDeadband ? speed : 0.0;
      arm.setSpeed(speed);
    }, arm).unless(arm::isEnabled));


    // m_Chooser.setDefaultOption("3rd Stage Cube Balance", autoRoutines.chargingStation());
    // m_Chooser.addOption("3rd Stage Cone Balance", autoRoutines.Cone3rdBalance());
    // m_Chooser.addOption("2nd Stage Cone Balance", autoRoutines.coneChargingStation());
    // m_Chooser.addOption("2nd Stage Cube Balance", autoRoutines.Cube2ndBalance());

    // m_Chooser.addOption("3rd Stage Cube", autoRoutines.cube3rdAuto());
    // m_Chooser.addOption("3rd Stage Cone", autoRoutines.cone3rdAuto());
    // m_Chooser.addOption("2nd Stage Cone", autoRoutines.cone2ndAuto());
    // m_Chooser.addOption("2nd Stage Cube", autoRoutines.cube2ndAuto());

    // m_Chooser.addOption("2 Piece Auto", autoRoutines.leftConeCubeAuto());



    SmartDashboard.putData(m_Chooser);

    configureButtonBindings();
  }

  private void startDashboard() {
    Dashboard.Swerve.Debugging.set(DashbaordConstants.SwerveDebugging);
    Dashboard.Swerve.Driver.set(DashbaordConstants.SwerveDriver);
    Dashboard.Elevator.Debugging.set(DashbaordConstants.ElevatorDebugging);
    Dashboard.Elevator.Driver.set(DashbaordConstants.ElevatorDriver);
    Dashboard.Intake.Debugging.set(DashbaordConstants.IntakeDebugging);
    Dashboard.Intake.Driver.set(DashbaordConstants.IntakeDriver);
    Dashboard.Auto.Debugging.set(DashbaordConstants.AutoDebugging);
    Dashboard.Auto.Driver.set(DashbaordConstants.AutoDriver);
    Dashboard.Tele.Debugging.set(DashbaordConstants.TeleDebugging);
    Dashboard.Tele.Driver.set(DashbaordConstants.TeleDriver);
    Dashboard.Limelight.Debugging.set(DashbaordConstants.LimelightDebugging);
    Dashboard.Limelight.Driver.set(DashbaordConstants.LimelightDriver);
    Dashboard.Arm.Debugging.set(DashbaordConstants.ArmDebugging);
    Dashboard.Arm.Driver.set(DashbaordConstants.ArmDriver);
  }


  private void configureButtonBindings() {

    driverJoystick.y().onTrue(Commands.runOnce(() -> swerve.reset(), swerve));
    driverJoystick.b().onTrue(Commands.runOnce(() -> swerve.toggleField(), swerve));

    // driverJoystick.x().onTrue(macros.scoreCone3rdStage());
    driverJoystick.leftBumper().onTrue(Commands.runOnce(() -> swerve.stopModules(), swerve));
    

    // driverJoystick.a().onTrue(new Balance(swerve));
    driverJoystick.rightBumper().onTrue(Commands.runOnce(swerve::stopModules, swerve));

    // // Right Trigger --> manual override
    operatorJoystick.rightTrigger().onTrue(
      Commands.runOnce(() -> {
        arm.stop();
        arm.disable();
        elevator.stop();
        elevator.disable();
      }, arm, elevator));


    // operatorJoystick.a().onTrue(
    //   arm.goToPosition(0)
    // );

    // operatorJoystick.b().onTrue(
    //   arm.goToPosition(1)
    // );

    // operatorJoystick.x().onTrue(
    //   arm.goToPosition(-0.3)
    // );
    // // Start --> Home
    // operatorJoystick.start().onTrue(macros.home());

    
    // // POV Left --> First Stage / Ground intake height
    // operatorJoystick.povLeft().onTrue(
    //   macros.groundIntake()
    // );

    // // POV Down --> Stowe
    // operatorJoystick.povDown().onTrue(
    //   macros.stow()
    // );

    // // POV Up --> Substration intake height
    // operatorJoystick.povUp().onTrue(
    //   macros.substationIntake()
    // );

    // // X --> Cone 2nd Stage
    // operatorJoystick.x().onTrue(
    //   macros.cone2ndStage()
    // );

    // // Y --> Cone 3rd Stage
    // operatorJoystick.y().onTrue(
    //   macros.cone3rdStage()
    // );

    // A --> Cube 2nd Stage
    // operatorJoystick.a().onTrue(
    //   Commands.runOnce(() -> { intake.setCone(); }, intake)
    // );

    // // B --> Cube 3rd Stage
    // operatorJoystick.b().onTrue(
    //   Commands.runOnce(() -> { intake.setCone(); }, intake)
    // );
    
    // Right Bumper --> Intake 
    // operatorJoystick.rightBumper().onTrue(
    //   macros.intake()
    // );

    // // Left Bumper --> Outtake
    // operatorJoystick.leftBumper().onTrue(
    //   macros.outtake()
    // );

    // operatorJoystick.rightBumper().onTrue(new IntakePiece(intake));
    // operatorJoystick.leftBumper().onTrue(new ArmCharacterization(newArm, operatorJoystick));

    // Back --> Manual Intake Stop
    // operatorJoystick.back().onTrue(Commands.runOnce(() -> intake.stop(), intake));

    // driverJoystick.a().onTrue(Commands.run(() -> swerve.drive(1.00, 0.000, 0.000), swerve));

  }

  public Command getAutonomousCommand() {
    // return m_Chooser.getSelected();
    return autoRoutines.debugOdometryReset(new Pose2d());
    // return autoRoutines.leftConeCubeAuto();
  }


}
