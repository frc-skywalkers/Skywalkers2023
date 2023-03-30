// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Dashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.NewArmConstants;

public class ProfiledPIDArmNew extends ProfiledPIDSubsystem {

  private final WPI_TalonFX armMotor = new WPI_TalonFX(ArmConstants.kArmPort, "CANivore");

  // WPI_TalonFX armMotor = new WPI_TalonFX(ArmConstants.kArmPort, "CANivore");
  private final CANCoder absoluteEncoder = new CANCoder(ArmConstants.kArmAbsoluteEncoderPort, "CANivore");

  public ProfiledPIDArmNew() {
    super(

        new ProfiledPIDController(
            NewArmConstants.kPArm,
            NewArmConstants.kIArm,
            NewArmConstants.kDArm,

            new TrapezoidProfile.Constraints(1.5, 2)));

    this.getController().setTolerance(0.03);

    armMotor.configFactoryDefault();
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    armMotor.setInverted(ArmConstants.kArmInverted);
    // armMotor.setIdleMode(NeutralMode.Brake);
    armMotor.setNeutralMode(NeutralMode.Brake);

    // armMotor.setSelectedSensorPosition(0);

    absoluteEncoder.configSensorDirection(NewArmConstants.kArmAbsEncoderInverted);
    // absoluteEncoder.configMagnetOffset(NewArmConstants.kAbsEncoderOffset);
    absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    resetEncoders();
    disable();

    SmartDashboard.putString("Working", "yes");
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
    Dashboard.Arm.Debugging.putNumber("Absolute Arm Position", getPosition());
    // Dashboard.Arm.Debugging.putNumber("Abs Enc Deg", absoluteEncoder.getAbsolutePosition());
    Dashboard.Arm.Debugging.putNumber("New Arm Position", getPosition());
    Dashboard.Arm.Debugging.putNumber("New Arm Velocity", getVelocity());
    // Dashboard.Arm.Debugging.putNumber("Arm Output Voltage", armMotor.getMotorOutputVoltage());
    Dashboard.Arm.Driver.putBoolean("New Arm Goal Reached", atGoal());
    Dashboard.Arm.Debugging.putNumber("arm abs encoder", absoluteEncoder.getAbsolutePosition());

  }

  public CommandBase goToPosition(double position) {
    return Commands.runOnce(() -> {
      this.setGoal(position);
      this.enable();
    }, this).andThen(Commands.waitUntil(this::atGoal));
  }

  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -3, 3);
    armMotor.setVoltage(voltage);
    SmartDashboard.putNumber("arm motor voltage characterization", voltage);
  }

  public void setSpeed(double speed) {
    speed = MathUtil.clamp(speed, -ArmConstants.kMaxArmSpeed, ArmConstants.kMaxArmSpeed);
    armMotor.set(speed + 0.0155);
    Dashboard.Arm.Debugging.putNumber("new arm speed", speed);
  }

  public double getPosition() {
    SmartDashboard.putNumber("stupid ass shit", (absoluteEncoder.getAbsolutePosition() - NewArmConstants.kAbsEncoderOffset));
    SmartDashboard.putNumber("stupid ass shit 1", (absoluteEncoder.getAbsolutePosition()));
    SmartDashboard.putNumber("stupid ass shit 2", (absoluteEncoder.getAbsolutePosition() + 99.6));
    // SmartDashboard.putNumber("stupid ass shit", (absoluteEncoder.getAbsolutePosition() - NewArmConstants.kAbsEncoderOffset));
    return (absoluteEncoder.getAbsolutePosition() - NewArmConstants.kAbsEncoderOffset) * Math.PI / 180.0;
    // return absoluteEncoder.getAbsolutePosition() * Math.PI / 180.0;
    
    // return absoluteEncoder.getSe
    // return armMotor.getSelectedSensorPosition() * NewArmConstants.kPositionConversionFactor;
  }

  public double getVelocity() {
    // return absoluteEncoder.getVelocity() * Math.PI / 180.0;
    return armMotor.getSelectedSensorVelocity() * NewArmConstants.kVelocityConversionFactor;
  }

  public void stop() {
    armMotor.stopMotor();
  }

  public void resetEncoders() {
    // armMotor.setSelectedSensorPosition(armEncoder.getPosition());
    armMotor.setSelectedSensorPosition((absoluteEncoder.getAbsolutePosition() - NewArmConstants.kAbsEncoderOffset) / 360.00 / NewArmConstants.kPositionConversionFactor);
    SmartDashboard.putNumber("debugging absolute encoder", getPosition());
  }

  public boolean isZeroed() {
    // return armMotor.getSupplyCurrent() > ArmConstants.kCurrentThreshold;
    return false;
  }

  public boolean atGoal() {
    // Dashboard.Arm.Driver.putBoolean("Arm Goal Reached", this.getController().atGoal());
    return Math.abs(getPosition() - this.getController().getGoal().position) <= 0.07;
  }
}
