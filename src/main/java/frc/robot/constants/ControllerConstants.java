package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 1;
    public static final int OPERATOR_CONTROLLER_PORT = 2;

    private static final CommandXboxController controller = new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);

    public static final Trigger climbUpButton = controller.povUp();
    public static final Trigger climbDownButton = controller.povDown();
    public static final Trigger intakeButton = controller.rightBumper();
    public static final Trigger outtakeButton = controller.leftBumper();
    public static final Trigger shooterButton = controller.rightTrigger();
    public static final Trigger ampButton = controller.leftTrigger();
    public static final Trigger autoIntakeButton = controller.a();
    public static final Trigger autoSpeakerButton = controller.b();
    public static final Trigger autoClimbButton = controller.x();
    public static final Trigger autoAmpButton = controller.y();

    public static final double DEADZONE = 0.1;

}
