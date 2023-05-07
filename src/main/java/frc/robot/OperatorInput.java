package frc.robot;

import com.torontoCodingCollective.TccGameController;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants.AutoPattern;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.drive.DriveOnHeadingCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The DriverController exposes all driver functions
 * <p>
 * Extend SubsystemBase in order to have a built in periodic call to support SmartDashboard updates
 */
public class OperatorInput extends SubsystemBase {

    public enum Stick {
        LEFT, RIGHT
    }

    public enum Axis {
        X, Y
    }

    private final TccGameController driverController;
    private final TccGameController operatorController;

    // Auto Setup Choosers
    SendableChooser<AutoPattern>    autoPatternChooser = new SendableChooser<>();
    SendableChooser<Integer>        waitTimeChooser    = new SendableChooser<>();
    SendableChooser<DriveMode>      driveModeChooser   = new SendableChooser<>();

    /**
     * Construct an OperatorInput class that is fed by a DriverController and an OperatorController.
     *
     * @param driverControllerPort on the driver station which the driver joystick is plugged into
     * @param operatorControllerPort on the driver station which the aux joystick is plugged into
     */
    public OperatorInput(int driverControllerPort, int operatorControllerPort) {

        driverController   = new TccGameController(driverControllerPort);
        operatorController = new TccGameController(operatorControllerPort);

        initializeDashboardSelectors();
    }

    /**
     * Configure the button bindings for all operator commands
     * <p>
     * NOTE: This routine requires all subsystems to be passed in
     * <p>
     * NOTE: This routine must only be called once from the RobotContainer
     *
     * @param driveSubsystem
     */
    public void configureButtonBindings(DriveSubsystem driveSubsystem) {

        // Cancel Command - cancels all running commands on all subsystems
        new Trigger(() -> driverController.getStartButton() || operatorController.getStartButton())
            .onTrue(new CancelCommand(driveSubsystem));

        // Gyro and Encoder Reset
        new Trigger(() -> driverController.getBackButton())
            .onTrue(new InstantCommand(() -> {
                driveSubsystem.resetGyro();
                driveSubsystem.resetEncoders();
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

    /**
     * Get the selected auto pattern
     *
     * @return AutoPattern
     */
    public AutoPattern getSelectedAuto() {
        return autoPatternChooser.getSelected();
    }

    /**
     * Get the selected drive mode
     *
     * @return DriveMode
     */
    public DriveMode getSelectedDriveMode() {
        return driveModeChooser.getSelected();
    }

    /**
     * Get the Auto wait time
     * <p>
     * The robot will wait this amount of time before starting auto
     *
     * @return wait time in seconds
     */
    public Integer getSelectedWaitTime() {
        return waitTimeChooser.getSelected();
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Driver Controller", driverController.toString());
        SmartDashboard.putString("Operator Controller", operatorController.toString());
    }

    /*
     * Support for haptic feedback to the driver
     */
    public void startVibrate() {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    }

    public void stopVibrate() {
        driverController.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    }

    private void initializeDashboardSelectors() {

        // Initialize the dashboard selectors
        autoPatternChooser.setDefaultOption("Do Nothing", AutoPattern.DO_NOTHING);
        SmartDashboard.putData("Auto Pattern", autoPatternChooser);
        autoPatternChooser.addOption("Drive Forward", AutoPattern.DRIVE_FORWARD);

        waitTimeChooser.setDefaultOption("No wait", 0);
        SmartDashboard.putData("Auto Wait Time", waitTimeChooser);
        waitTimeChooser.addOption("1 second", 1);
        waitTimeChooser.addOption("3 seconds", 3);
        waitTimeChooser.addOption("5 seconds", 5);

        driveModeChooser.setDefaultOption("Arcade", DriveMode.ARCADE);
        SmartDashboard.putData("Drive Mode", driveModeChooser);
        driveModeChooser.addOption("Tank", DriveMode.TANK);
        driveModeChooser.addOption("Single Stick (L)", DriveMode.SINGLE_STICK_LEFT);
        driveModeChooser.addOption("Single Stick (R)", DriveMode.SINGLE_STICK_RIGHT);
    }


}
