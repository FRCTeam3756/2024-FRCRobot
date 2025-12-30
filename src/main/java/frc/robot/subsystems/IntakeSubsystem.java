// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  public final VictorSPX intakeMotor;
  private static final double INTAKE_SPEED = 1.0;
  private static final double OUTTAKE_SPEED = -1.0;

  private static final int MOTOR_CAN_ID = 21;

  public IntakeSubsystem() {
    intakeMotor = new VictorSPX(MOTOR_CAN_ID);
  }

  public void intakeGamePiece() {
    intakeMotor.set(ControlMode.PercentOutput, INTAKE_SPEED);
  }

  public void outtakeGamePiece() {
    intakeMotor.set(ControlMode.PercentOutput, OUTTAKE_SPEED);
  }

  public void stop() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    
  }
}