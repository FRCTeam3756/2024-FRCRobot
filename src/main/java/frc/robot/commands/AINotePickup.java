package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DSVisionSubsystem;

public class AINotePickup extends Command {
    private final DriveSubsystem swerveSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final DSVisionSubsystem dsVisionSubsystem;

    public AINotePickup(DriveSubsystem swerveSubsystem, IntakeSubsystem intakeSubsystem, DSVisionSubsystem dsVisionSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.dsVisionSubsystem = dsVisionSubsystem;

        addRequirements(swerveSubsystem, intakeSubsystem);
    }

    @Override
    public void execute() {
        performDriveAction();
        performIntakeAction();
    }

    public void performDriveAction() {
        double[] driveCommand = dsVisionSubsystem.getDriveDirections();
        double speed = (driveCommand.length > 0) ? driveCommand[0] : 0.0;
        double angle = (driveCommand.length > 1) ? driveCommand[1] : 0.0;

        swerveSubsystem.drive(speed, angle);
    }

    private void performIntakeAction() {
        boolean intakeCommand = dsVisionSubsystem.getIntakeCommand();

        if (intakeCommand) {
            intakeSubsystem.intakeGamePiece();
        } else {
            intakeSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        intakeSubsystem.stop();
    }
}
