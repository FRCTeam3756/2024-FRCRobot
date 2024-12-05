// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
  public static final double MAX_MODULE_VELOCITY = 4.5;
  public static final double MAX_ROBOT_VELOCITY = 4.5;
  public static final double MAX_ROBOT_RADIAN_VELOCITY = 8.0;

  public static final double TRACK_WIDTH = Units.inchesToMeters(28);
  public static final double WHEEL_BASE = Units.inchesToMeters(32);
  public static final Rotation2d NAVX_ANGLE_OFFSET = Rotation2d.fromDegrees(90);

  public static final class ModuleIndices {
    public static final int FRONT_LEFT = 0;
    public static final int REAR_LEFT = 1;
    public static final int FRONT_RIGHT = 2;
    public static final int REAR_RIGHT = 3;
  }

  public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
      new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
      new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
      new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0));

  public static final double XY_SPEED_LIMIT = 1.0;
  public static final double Z_SPEED_LIMIT = 1.0;

}
