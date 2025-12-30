package frc.robot.constants;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;
// import com.pathplanner.lib.util.

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {

    public static final double TURBO_DRIVE_MULTIPLIER = 0.15; // 1 is fast //TODO: Fix
    public static final double SLOW_DRIVE_MULTIPLIER = 0.1; // Half is appropriate

    public static final double TRACK_WIDTH = Units.inchesToMeters(28);
    public static final double WHEEL_BASE = Units.inchesToMeters(32);
    public static final Translation2d FLMODULEOFFSET = new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
    public static final Translation2d FRMODULEOFFSET = new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0);
    public static final Translation2d BLMODULEOFFSET = new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
    public static final Translation2d BRMODULEOFFSET = new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0);

    public static final double MAX_MODULE_SPEED = 4.8; // M/S

    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double STEERING_GEAR_RATIO = 1.d / (150d / 7d);
    public static final double DRIVE_GEAR_RATIO = 1.d / 6.75d;

    public static final double DRIVE_ROTATION_TO_METER = DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER;
    public static final double STEER_ROTATION_TO_RADIANS = STEERING_GEAR_RATIO * Math.PI * 2d;
    public static final double DRIVE_METERS_PER_MINUTE = DRIVE_ROTATION_TO_METER / 60d;
    public static final double STEER_RADIANS_PER_MINUTE = STEER_ROTATION_TO_RADIANS / 60d;

    public static final double MODULE_KP = 0.30;
    public static final double MODULE_KD = 0.0;

    // --------- Front Left Module --------- \\
    public static final int FL_DRIVE_ID = 2;
    public static final int FL_STEER_ID = 6;
    public static final int FL_ABSOLUTE_ENCODER_PORT = 10;
    public static final double FL_OFFSET_RADIANS = Units.rotationsToRadians(0.335205 - .25);
    public static final boolean FL_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean FL_MOTOR_REVERSED = false;

    // --------- Front Right Module --------- \\
    public static final int FR_DRIVE_ID = 4;
    public static final int FR_STEER_ID = 8;
    public static final int FR_ABSOLUTE_ENCODER_PORT = 12;
    public static final double FR_OFFSET_RADIANS = Units.rotationsToRadians(-0.215576 + .25);
    public static final boolean FR_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean FR_MOTOR_REVERSED = false;

    // --------- Back Right Module --------- \\
    public static final int BR_DRIVE_ID = 3;
    public static final int BR_STEER_ID = 7;
    public static final int BR_ABSOLUTE_ENCODER_PORT = 11;
    public static final double BR_OFFSET_RADIANS = Units.rotationsToRadians(0.067871 + .25);
    public static final boolean BR_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean BR_MOTOR_REVERSED = true;

    // --------- Back Left Module --------- \\
    public static final int BL_DRIVE_ID = 1;
    public static final int BL_STEER_ID = 5;
    public static final int BL_ABSOLUTE_ENCODER_PORT = 9;
    public static final double BL_OFFSET_RADIANS = Units.rotationsToRadians(0.273193 - .25);
    public static final boolean BL_ABSOLUTE_ENCODER_REVERSED = true;
    public static final boolean BL_MOTOR_REVERSED = false;
}
