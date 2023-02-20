// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

  WPI_TalonFX armMotor = new WPI_TalonFX(ArmConstants.kArmPort, "CANivore");

  public ArmSubsystem() {
    armMotor.configFactoryDefault();
    armMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    armMotor.setInverted(ArmConstants.kArmInverted);
    armMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setVoltage(double voltage) {
    armMotor.setVoltage(voltage);
  }

  public void setSpeed(double speed) {
    speed = MathUtil.clamp(speed, -ArmConstants.kMaxArmSpeed, ArmConstants.kMaxArmSpeed);
    armMotor.set(speed);
  }

  public double getPosition() {
    return armMotor.getSelectedSensorPosition() * ArmConstants.kConversionFactor;
  }

  public double getVelocity() {
    return armMotor.getSelectedSensorVelocity() * ArmConstants.kConversionFactor;
  }

  public void stop() {
    armMotor.stopMotor();
  }

  public void resetEncoders() {
    armMotor.setSelectedSensorPosition(0);
  }

  public boolean isZeroed() {
    return armMotor.getSupplyCurrent() > ArmConstants.kCurrentThreshold;
  }

  @Override
  public void periodic() {


  }
}
