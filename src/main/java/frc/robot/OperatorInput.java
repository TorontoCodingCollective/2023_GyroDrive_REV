package frc.robot;

import com.torontoCodingCollective.TccGameController;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The DriverController exposes all driver functions
 */
public class OperatorInput extends SubsystemBase {

    private final TccGameController driverController;
    private final TccGameController operatorController;

    public enum Stick {
        LEFT, RIGHT
    }

    public enum Axis {
        X, Y
    }

    /**
     * Construct an OperatorInput class that is fed by a DriverController and an OperatorController.
     *
     * @param driverControllerPort on the driver station which the driver joystick is plugged into
     * @param operatorControllerPort on the driver station which the aux joystick is plugged into
     */
    public OperatorInput(int driverControllerPort, int operatorControllerPort) {

        driverController   = new TccGameController(driverControllerPort);
        operatorController = new TccGameController(operatorControllerPort);
    }

    /*
     * The following routines are used by the default commands for each subsystem
     *
     * They allow the default commands to get user input to manually move the
     * robot elements.
     */

    public boolean isBoost() {
        return driverController.getLeftBumper();
    }

    public boolean isSlowDown() {
        return driverController.getRightBumper();
    }

    public double getDriverControllerAxis(Stick stick, Axis axis) {

        switch (stick) {

        case LEFT:
            switch (axis) {
            case X:
                return driverController.getLeftX();
            case Y:
                return driverController.getLeftY();
            }
            break;

        case RIGHT:
            switch (axis) {
            case X:
                return driverController.getRightX();
            case Y:
                return driverController.getRightY();
            }
            break;
        }

        return 0;
    }

    /*
     * Support for haptic feedback
     */
    public void startVibrate() {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }

    public void stopVibrate() {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }

    /**
     * Configure the button bindings for all operator commands
     * <p>
     * NOTE: This routine requires all subsystems to be passed in
     *
     * @param driveSubsystem
     */
    public void configureButtonBindings(DriveSubsystem driveSubsystem) {

        // Cancel Command - cancels all running commands on all subsystems
        new Trigger(() -> driverController.getStartButton() || operatorController.getStartButton())
            .onTrue(new CancelCommand(driveSubsystem));

        // Gyro Reset
        new Trigger(() -> driverController.getBackButton())
            .onTrue(new InstantCommand(() -> {
                driveSubsystem.resetGyro();
            }));

        // Configure the DPAD to drive one meter on a heading
        new Trigger(() -> driverController.getPOV() == 0)
            .onTrue(new DriveOnHeadingCommand(0, .5, 100, driveSubsystem));

        new Trigger(() -> driverController.getPOV() == 90)
            .onTrue(new DriveOnHeadingCommand(90, .5, 100, driveSubsystem));

        new Trigger(() -> driverController.getPOV() == 180)
            .onTrue(new DriveOnHeadingCommand(180, .5, 100, driveSubsystem));

        new Trigger(() -> driverController.getPOV() == 270)
            .onTrue(new DriveOnHeadingCommand(270, .5, 100, driveSubsystem));
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Driver Controller", driverController.toString());
        SmartDashboard.putString("Operator Controller", operatorController.toString());
    }
}
