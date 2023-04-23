// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.Constants.OperatorInputConstants;
import frc.robot.commands.auto.DriveForwardAutoCommand;
import frc.robot.commands.drive.DefaultDriveCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    private final DriveSubsystem driveSubsystem     = new DriveSubsystem();

    // Auto Setup Choosers
    SendableChooser<AutoPattern> autoPatternChooser = new SendableChooser<>();

    // Driver and operator controllers
    private final OperatorInput  operatorInput      = new OperatorInput(
        OperatorInputConstants.DRIVER_CONTROLLER_PORT,
        OperatorInputConstants.OPERATOR_CONTROLLER_PORT);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Initialize all Subsystem default commands.
        driveSubsystem.setDefaultCommand(
            new DefaultDriveCommand(operatorInput, driveSubsystem));

        // Initialize the autonomous choosers
        initAutoSelectors();

        // Configure the button bindings
        operatorInput.configureButtonBindings(driveSubsystem);

    }

    private void initAutoSelectors() {

        autoPatternChooser.setDefaultOption("Do Nothing", AutoPattern.DO_NOTHING);
        SmartDashboard.putData("Auto Pattern", autoPatternChooser);
        autoPatternChooser.addOption("Drive Forward", AutoPattern.DRIVE_FORWARD);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        switch (autoPatternChooser.getSelected()) {

        case DRIVE_FORWARD:
            return new DriveForwardAutoCommand(
                driveSubsystem);

        default:
            // If the chooser did not work, then do nothing as the default auto.
            return new InstantCommand();

        }
    }
}
