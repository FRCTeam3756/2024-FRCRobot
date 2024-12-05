package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.MAX_ROBOT_VELOCITY / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                        SwerveConstants.STEER_RADIANS_PER_MINUTE / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
}