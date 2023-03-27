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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Dashboard;
import frc.robot.Constants.ArmConstants;

public class ProfiledPIDArm extends ProfiledPIDSubsystem {

  WPI_TalonFX armMotor = new WPI_TalonFX(ArmConstants.kArmPort, "CANivore");
  private final CANCoder absoluteEncoder = new CANCoder(ArmConstants.kArmAbsoluteEncoderPort, "CANivore");

  public ProfiledPIDArm() {
    super(

        new ProfiledPIDController(
            ArmConstants.kPArm,
            ArmConstants.kIArm,
            ArmConstants.kDArm,

            new TrapezoidProfile.Constraints(2.75, 3.50)));

    this.getController().setTolerance(0.03);

    armMotor.configFactoryDefault();
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    armMotor.setInverted(ArmConstants.kArmInverted);
    armMotor.setNeutralMode(NeutralMode.Brake);
    absoluteEncoder.configSensorDirection(ArmConstants.kArmAbsEncoderInverted);
    absoluteEncoder.configMagnetOffset(ArmConstants.kAbsEncoderOffset);
    absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    // armMotor.configForwardSoftLimitThreshold(1.35 / ArmConstants.kPositionConversionFactor);
    // armMotor.configForwardSoftLimitEnable(true);

    resetEncoders();
    disable();

    Dashboard.Arm.Debugging.putNumber("Desired Arm Position", 0);
    Dashboard.Arm.Debugging.putNumber("Desired Arm Velocity", 0);
    
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {

    double feedforward = 0.000;

    if(setpoint.velocity > 0) {
      feedforward = setpoint.velocity * ArmConstants.kVUp + ArmConstants.kSUp;
    }
    else {
      feedforward = setpoint.velocity * ArmConstants.kVDown + ArmConstants.kSDown;
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
    Dashboard.Arm.Debugging.putNumber("Absolute Arm Position", absoluteEncoder.getAbsolutePosition() * 2 * Math.PI /360.0);
    Dashboard.Arm.Debugging.putNumber("Abs Enc Deg", absoluteEncoder.getAbsolutePosition());
    Dashboard.Arm.Debugging.putNumber("Arm Position", getPosition());
    Dashboard.Arm.Debugging.putNumber("Arm Velocity", getVelocity());
    Dashboard.Arm.Debugging.putNumber("Arm Output Voltage", armMotor.getMotorOutputVoltage());
    Dashboard.Arm.Driver.putBoolean("Arm Goal Reached", this.getController().atGoal());
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
    speed = MathUtil.clamp(speed, -ArmConstants.kMaxArmSpeed, ArmConstants.kMaxArmSpeed);
    armMotor.set(speed);
  }

  public double getPosition() {
    return absoluteEncoder.getAbsolutePosition() * Math.PI / 180.0;
  }

  public double getVelocity() {
    return absoluteEncoder.getVelocity() * Math.PI / 180.0;
  }

  public void stop() {
    armMotor.stopMotor();
  }

  public void resetEncoders() {
    armMotor.setSelectedSensorPosition(absoluteEncoder.getAbsolutePosition());
  }

  public boolean isZeroed() {
    return armMotor.getSupplyCurrent() > ArmConstants.kCurrentThreshold;
  }

  public boolean atGoal() {
    // Dashboard.Arm.Driver.putBoolean("Arm Goal Reached", this.getController().atGoal());
    return Math.abs(getPosition() - this.getController().getGoal().position) <= 0.07;
  }
}
