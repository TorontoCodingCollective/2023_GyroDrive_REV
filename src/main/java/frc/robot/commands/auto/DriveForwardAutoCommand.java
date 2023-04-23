package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.subsystems.DriveSubsystem;


public class DriveForwardAutoCommand extends SequentialCommandGroup {

    public DriveForwardAutoCommand(DriveSubsystem driveSubsystem) {

        StringBuilder sb = new StringBuilder("Auto Selections: ");
        sb.append("Pattern: Drive Forward Auto Pattern");
        System.out.println(sb.toString());

        // Set the gyro angle to zero
        // use an Instant command to call the method in the subsystem
        addCommands(new InstantCommand(() -> {
            driveSubsystem.setGyroHeading(0);
        }));

        // Wait the selected number of seconds
        addCommands(new WaitCommand(1));

        // DriveForward
        addCommands(new DriveOnHeadingCommand(0, .5, 100, driveSubsystem));

    }
}
