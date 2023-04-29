package frc.robot.commands.drive;

import com.torontoCodingCollective.TccCommandBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMode;
import frc.robot.OperatorInput;
import frc.robot.OperatorInput.Axis;
import frc.robot.OperatorInput.Stick;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends TccCommandBase {

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

        double leftSpeed = 0, rightSpeed = 0;

        // If this is a tank drive robot, then the left and right speeds are set from the
        // joystick values.
        if (driveMode == DriveMode.TANK) {

            leftSpeed  = operatorInput.getDriverControllerAxis(Stick.LEFT, Axis.Y) * driveScalingFactor;
            rightSpeed = operatorInput.getDriverControllerAxis(Stick.RIGHT, Axis.Y) * driveScalingFactor;

            // The max differential between the left and right should not exceed the scaling factor
            if (Math.abs(leftSpeed - rightSpeed) > driveScalingFactor) {
                double avgSpeed = (leftSpeed + rightSpeed) / 2.0d;
                if (leftSpeed < avgSpeed) {
                    leftSpeed  = avgSpeed - (0.5 * driveScalingFactor);
                    rightSpeed = avgSpeed + (0.5 * driveScalingFactor);
                }
                else {
                    leftSpeed  = avgSpeed + (0.5 * driveScalingFactor);
                    rightSpeed = avgSpeed - (0.5 * driveScalingFactor);
                }
            }
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

            speed *= driveScalingFactor;
            turn  *= driveScalingFactor / 2.0;

            double turnAdjustmentPerSide = turn / 2.0;

            // When turning, keep the differential between the left and right while
            // maximizing the speed. The maximum speed for any side is the driveScalingFactor
            if (Math.abs(speed) + Math.abs(turnAdjustmentPerSide) > driveScalingFactor) {
                speed = (driveScalingFactor - Math.abs(turnAdjustmentPerSide)) * Math.signum(speed);
            }

            leftSpeed  = speed + turnAdjustmentPerSide;
            rightSpeed = speed - turnAdjustmentPerSide;
        }

        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);
    }

    @Override
    public boolean isFinished() {
        return false; // default commands never end but can be interrupted
    }

    @Override
    public void end(boolean interrupted) {

        logCommandEnd(interrupted);
        driveSubsystem.setMotorSpeeds(0, 0);
    }
}