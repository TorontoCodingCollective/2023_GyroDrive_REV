package frc.robot.commands.drive;

import com.torontoCodingCollective.TccCommandBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.OperatorInput;
import frc.robot.OperatorInput.Axis;
import frc.robot.OperatorInput.Stick;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends TccCommandBase {

    private class DriveSpeed {

        double left, right;

        private DriveSpeed(double left, double right) {
            this.left  = left;
            this.right = right;
        }
    }

    private final DriveSubsystem driveSubsystem;
    private final OperatorInput  operatorInput;

    /**
     * Creates a new ExampleCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public DefaultDriveCommand(OperatorInput operatorInput, DriveSubsystem driveSubsystem) {

        this.operatorInput  = operatorInput;
        this.driveSubsystem = driveSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {

        // Get the selected drive mode
        DriveMode driveMode          = operatorInput.getSelectedDriveMode();

        // Calculate the drive scaling factor based on the boost mode and the slow mode.
        double    driveScalingFactor = DriveConstants.DRIVE_SCALING_NORMAL;

        if (operatorInput.isBoost()) {
            driveScalingFactor = DriveConstants.DRIVE_SCALING_BOOST;
        }
        if (operatorInput.isSlowDown()) {
            driveScalingFactor = DriveConstants.DRIVE_SCALING_SLOW;
        }

        DriveSpeed driveSpeed = null;

        // If this is a tank drive robot, then the left and right speeds are set from the
        // joystick values.
        if (driveMode == DriveMode.TANK) {

            driveSpeed = calcDriveSpeed_Tank(
                operatorInput.getDriverControllerAxis(Stick.LEFT, Axis.Y),
                operatorInput.getDriverControllerAxis(Stick.RIGHT, Axis.Y),
                driveScalingFactor);

        }
        else {
            // One of the arcade style drive modes.

            double speed = 0, turn = 0;

            switch (driveMode) {

            case ARCADE:
                speed = operatorInput.getDriverControllerAxis(Stick.LEFT, Axis.Y);
                turn = operatorInput.getDriverControllerAxis(Stick.RIGHT, Axis.X);
                break;

            case SINGLE_STICK_LEFT:
                speed = operatorInput.getDriverControllerAxis(Stick.LEFT, Axis.Y);
                turn = operatorInput.getDriverControllerAxis(Stick.LEFT, Axis.X);
                break;

            case SINGLE_STICK_RIGHT:
                speed = operatorInput.getDriverControllerAxis(Stick.RIGHT, Axis.Y);
                turn = operatorInput.getDriverControllerAxis(Stick.RIGHT, Axis.X);
                break;

            default:
                break;
            }

            driveSpeed = calcDriveSpeed_Arcade(speed, turn, driveScalingFactor);

        }

        driveSubsystem.setMotorSpeeds(driveSpeed.left, driveSpeed.right);
    }

    @Override
    public boolean isFinished() {
        return false; // default commands never end but can be interrupted
    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);
    }

    /**
     * Calculate the scaled tank drive speeds from the passed in values. In tank mode, the differential between
     * the left and right sides should not exceed the drive forward speed. This will provide some greater control
     * over small turns while moving forward.
     * <p>
     *
     * @param leftStick value
     * @param rightStick value
     * @param driveScalingFactor
     * @return DriveSpeed object containing the left and right motor speeds to apply to the motors.
     */
    private DriveSpeed calcDriveSpeed_Tank(double leftStick, double rightStick, double driveScalingFactor) {

        // Translate to arcade drive, and call the arcade command
        double speed = (leftStick + rightStick) / 2.0d;
        double turn  = (leftStick - rightStick) / 2.0d;

        return calcDriveSpeed_Arcade(speed, turn, driveScalingFactor);
    }

    /**
     * Calculate the scaled arcade drive speeds from the passed in values. In arcade mode, the turn
     * is cut in half to help control the robot more consistently.
     *
     * @param speed
     * @param turn
     * @param driveScalingFactor
     * @return DriveSpeed object containing the left and right motor speeds to apply to the motors.
     */
    private DriveSpeed calcDriveSpeed_Arcade(double speed, double turn, double driveScalingFactor) {

        speed *= driveScalingFactor;

        // Slow the turn by half in order to make the robot more controllable, and to
        // limit the max spin-on-the-spot speed.
        turn  *= driveScalingFactor / 2.0;

        double turnAdjustmentPerSide = turn / 2.0;

        // When turning, keep the differential between the left and right while
        // maximizing the speed. The maximum speed for any side is the driveScalingFactor
        if (Math.abs(speed) + Math.abs(turnAdjustmentPerSide) > driveScalingFactor) {
            speed = (driveScalingFactor - Math.abs(turnAdjustmentPerSide)) * Math.signum(speed);
        }

        return new DriveSpeed(speed + turnAdjustmentPerSide, speed - turnAdjustmentPerSide);
    }

}