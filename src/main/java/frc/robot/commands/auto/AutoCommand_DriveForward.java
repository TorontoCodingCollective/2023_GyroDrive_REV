package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.subsystems.DriveSubsystem;


public class AutoCommand_DriveForward extends SequentialCommandGroup {

    public AutoCommand_DriveForward(double waitTime, DriveSubsystem driveSubsystem) {

        System.out.println("Auto Pattern: Drive Forward");

        // Resolve the wait time before starting auto
        // The wait time here is inserted as an example of how to use multiple
        // selectors on the Shuffleboard Dashboard

        // Set the gyro angle to zero
        // use an Instant command to call the method in the subsystem
        addCommands(new InstantCommand(() -> {
            driveSubsystem.setGyroHeading(0);
        }));

        // Wait the selected number of seconds
        addCommands(new WaitCommand(waitTime));

        // DriveForward
        addCommands(new DriveOnHeadingCommand(0, .5, 100, driveSubsystem));

    }
}
