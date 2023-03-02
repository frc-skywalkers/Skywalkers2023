// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class ProfiledPIDArm extends ProfiledPIDSubsystem {

  WPI_TalonFX armMotor = new WPI_TalonFX(ArmConstants.kArmPort, "CANivore");

  public ProfiledPIDArm() {
    super(

        new ProfiledPIDController(
            ArmConstants.kPArm,
            ArmConstants.kIArm,
            ArmConstants.kDArm,

            new TrapezoidProfile.Constraints(0.75, 1.00)));

    armMotor.configFactoryDefault();
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    armMotor.setInverted(ArmConstants.kArmInverted);
    armMotor.setNeutralMode(NeutralMode.Brake);

    disable();

    SmartDashboard.putNumber("Desired Arm Position", 0);
    SmartDashboard.putNumber("Desired Arm Velocity", 0);
    offsetEncoders();
    
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
    SmartDashboard.putNumber("Set Arm Voltage", feedforward + output);
    SmartDashboard.putNumber("Desired Arm Position", setpoint.position);
    SmartDashboard.putNumber("Desired Arm Velocity", setpoint.velocity);
  }

  @Override
  public double getMeasurement() {
    return getPosition();
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Arm Position", getPosition());
    SmartDashboard.putNumber("Arm Velocity", getVelocity());
    SmartDashboard.putNumber("Arm Output Voltage", armMotor.getMotorOutputVoltage());
  }

  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -5, 5);
    armMotor.setVoltage(voltage);
  }

  public void setSpeed(double speed) {
    speed = MathUtil.clamp(speed, -ArmConstants.kMaxArmSpeed, ArmConstants.kMaxArmSpeed);
    armMotor.set(speed);
  }

  public double getPosition() {
    return armMotor.getSelectedSensorPosition() * ArmConstants.kPositionConversionFactor;
  }

  public double getVelocity() {
    return armMotor.getSelectedSensorVelocity() * ArmConstants.kVelocityConversionFactor * 10.000;
  }

  public void stop() {
    armMotor.stopMotor();
  }

  public void resetEncoders() {
    armMotor.setSelectedSensorPosition(0);
  }

  public void offsetEncoders() {
    armMotor.setSelectedSensorPosition(1.335 / ArmConstants.kPositionConversionFactor);
  }

  public boolean isZeroed() {
    return armMotor.getSupplyCurrent() > ArmConstants.kCurrentThreshold;
  }
}
