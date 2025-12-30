package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class AutoConstants {

    public static final double MAX_ACCELERATION = 3; // m/s^2
    public static final double P_X_CONTROLLER = 1.5;
    public static final double P_Y_CONTROLLER = 1.5;
    public static final double P_THETA_CONTROLLER = 3;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            SwerveConstants.STEER_RADIANS_PER_MINUTE / 10,
            Math.PI / 4);
}
