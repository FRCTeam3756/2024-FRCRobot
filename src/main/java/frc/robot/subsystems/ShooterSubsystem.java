// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private VictorSPX leftMotor;
  private TalonSRX rightMotor;

  public ShooterSubsystem() {
    leftMotor = new VictorSPX(ShooterConstants.LEFT_MOTOR_ID);
    rightMotor = new TalonSRX(ShooterConstants.RIGHT_MOTOR_ID);
  }

  public void shootSpeaker() {
    setMotorSpeeds(ShooterConstants.SPEAKER_SHOOT_SPEED);
  }

  public void shootAmp() {
    setMotorSpeeds(ShooterConstants.AMP_SHOOT_SPEED);
  }

  public void stop() {
    setMotorSpeeds(0.0);
  }

  private void setMotorSpeeds(double speed) {
    leftMotor.set(ControlMode.PercentOutput, -speed);
    rightMotor.set(ControlMode.PercentOutput, -speed); //TODO: Due to wiring issue, it is speed instead of -speed
  }

  @Override
  public void periodic() {
    
  }
}