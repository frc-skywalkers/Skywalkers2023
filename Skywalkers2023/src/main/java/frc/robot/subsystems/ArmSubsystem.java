// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Dashboard;
import frc.robot.Constants.NewArmConstants;

public class ArmSubsystem extends ProfiledPIDSubsystem {

  private final WPI_TalonFX armMotor = new WPI_TalonFX(NewArmConstants.kArmPort, "CANivore");

  // WPI_TalonFX armMotor = new WPI_TalonFX(ArmConstants.kArmPort, "CANivore");
  private final CANCoder absoluteEncoder = new CANCoder(NewArmConstants.kArmAbsoluteEncoderPort, "CANivore");

  public ArmSubsystem() {
    super(

        new ProfiledPIDController(
            NewArmConstants.kPArm,
            NewArmConstants.kIArm,
            NewArmConstants.kDArm,

            new TrapezoidProfile.Constraints(3, 4)));

    this.getController().setTolerance(0.03);

    armMotor.configFactoryDefault();
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    armMotor.setInverted(NewArmConstants.kArmInverted);
    armMotor.setNeutralMode(NeutralMode.Brake);

    absoluteEncoder.configSensorDirection(NewArmConstants.kArmAbsEncoderInverted);
    // absoluteEncoder.configMagnetOffset(NewArmConstants.kAbsEncoderOffset);
    absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    resetEncoders();
    disable();

    Dashboard.Arm.Debugging.putNumber("Desired Arm Position", 0);
    Dashboard.Arm.Debugging.putNumber("Desired Arm Velocity", 0);
    
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {

    double feedforward = 0.000;

    if(setpoint.velocity > 0) {
      feedforward = setpoint.velocity * NewArmConstants.kVUp + NewArmConstants.kSUp;
    }
    else {
      feedforward = setpoint.velocity * NewArmConstants.kVDown + NewArmConstants.kSDown;
    }

    setVoltage(feedforward + output);
    Dashboard.Arm.Debugging.putNumber("Set Arm Voltage", feedforward + output);
    Dashboard.Arm.Debugging.putNumber("Desired Arm Position", setpoint.position);
    Dashboard.Arm.Debugging.putNumber("Desired Arm Velocity", setpoint.velocity);
  }

  @Override
  public double getMeasurement() {
    return getPosition();
  }

  @Override
  public void periodic() {
    super.periodic();
    if (getPosition() > NewArmConstants.kTopLimit && armMotor.getMotorOutputVoltage() > 0) {
      armMotor.stopMotor();
    }
    if (getPosition() < NewArmConstants.kBottomLimit && armMotor.getMotorOutputVoltage() < 0) {
      armMotor.stopMotor();
    }
    
    Dashboard.Arm.Debugging.putNumber("New Arm Position", getPosition());
    Dashboard.Arm.Debugging.putNumber("New Arm Velocity", getVelocity());
    Dashboard.Arm.Debugging.putNumber("Arm Output Voltage", armMotor.getMotorOutputVoltage());
    Dashboard.Arm.Driver.putBoolean("New Arm Goal Reached", atGoal());
    Dashboard.Arm.Debugging.putNumber("Arm Abs Enc Deg", absoluteEncoder.getAbsolutePosition());

  }

  public CommandBase goToPosition(double position) {
    return Commands.runOnce(() -> {
      this.setGoal(position);
      this.enable();
    }, this).andThen(Commands.waitUntil(this::atGoal));
  }

  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -6, 6);
    armMotor.setVoltage(voltage);
  }

  public void setSpeed(double speed) {
    speed = MathUtil.clamp(speed, -NewArmConstants.kMaxArmSpeed, NewArmConstants.kMaxArmSpeed);
    armMotor.set(speed + 0.0155);
    if (getPosition() > NewArmConstants.kTopLimit && speed > 0) {
      armMotor.stopMotor();
    }
    if (getPosition() < NewArmConstants.kBottomLimit && speed < 0) {
      armMotor.stopMotor();
    }
    Dashboard.Arm.Debugging.putNumber("New Arm Speed", speed);
  }

  public double getPosition() {
    return (absoluteEncoder.getAbsolutePosition() - NewArmConstants.kAbsEncoderOffset) * Math.PI / 180.0;
  }

  public double getVelocity() {
    return armMotor.getSelectedSensorVelocity() * NewArmConstants.kVelocityConversionFactor;
  }

  public void stop() {
    armMotor.stopMotor();
  }

  public void resetEncoders() {
    armMotor.setSelectedSensorPosition((absoluteEncoder.getAbsolutePosition() - NewArmConstants.kAbsEncoderOffset) / 360.00 / NewArmConstants.kPositionConversionFactor);
  }

  public boolean atGoal() {
    return Math.abs(getPosition() - this.getController().getGoal().position) <= 0.07;
  }
}
