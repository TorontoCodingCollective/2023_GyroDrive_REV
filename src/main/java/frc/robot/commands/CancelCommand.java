package frc.robot.commands;

import com.torontoCodingCollective.TccCommandBase;

import frc.robot.subsystems.DriveSubsystem;

/**
 * This command is used to safely stop the robot in its current position, and to cancel any running
 * commands
 */
public class CancelCommand extends TccCommandBase {

    private final DriveSubsystem driveSubsystem;

    /**
     * Cancel the commands running on all subsystems.
     *
     * All subsystems must be passed to this command, and each subsystem should have a stop command
     * that safely stops the robot
     * from moving.
     */
    public CancelCommand(DriveSubsystem driveSubsystem) {

        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        logCommandStart();

        driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {

        // Wait 1/4 second before finishing.
        // Allow time for the robot to stop moving
        if (System.currentTimeMillis() - commandStartTimeMillis > 250) {
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }

}
