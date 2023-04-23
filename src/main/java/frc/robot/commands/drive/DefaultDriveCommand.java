package frc.robot.commands.drive;

import com.torontoCodingCollective.TccCommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OperatorInput;
import frc.robot.OperatorInput.Axis;
import frc.robot.OperatorInput.Stick;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends TccCommandBase {

    private final DriveSubsystem driveSubsystem;
    private final OperatorInput  driverController;

    /**
     * Creates a new ExampleCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public DefaultDriveCommand(OperatorInput operatorInput, DriveSubsystem driveSubsystem) {

        this.driverController = operatorInput;
        this.driveSubsystem   = driveSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        logCommandStart();
    }

    @Override
    public void execute() {

        // forwards/backwards speed
        double       speed   = driverController.getDriverControllerAxis(Stick.LEFT, Axis.Y);
        // turn speed
        final double rawTurn = driverController.getDriverControllerAxis(Stick.RIGHT, Axis.X);

        SmartDashboard.putNumber("Operator Speed", speed);
        SmartDashboard.putNumber("Operator Turn", rawTurn);

        double  turn      = rawTurn / 2;
        boolean boost     = driverController.isBoost();
        boolean slow      = driverController.isSlowDown();

        double  leftSpeed = 0, rightSpeed = 0;

        if (slow) {
            speed = speed / 5;
            turn  = turn / 2.5; // Turn was already divided by 2 above
        }

        else if (!boost) {
            speed = speed / 2;
        }

        else {
            speed = Math.signum(speed);
        }

        if (speed < 0) {
            turn = -turn;
        }

        if (speed == 0) {
            leftSpeed  = turn;
            rightSpeed = -turn;
        }
        else if (boost) {

            // Turning left
            if (rawTurn < 0) {

                // If boosted and at full speed, and the
                // turn is limited to 0.5, then the max turn
                // that can be achieved is 1.0 vs 0.5.
                leftSpeed  = speed + turn;
                rightSpeed = speed;
            }
            // Turning right
            else if (rawTurn > 0) {
                leftSpeed  = speed;
                rightSpeed = speed - turn;
            }
            else {
                leftSpeed  = speed;
                rightSpeed = speed;
            }
        }
        else {
            if (rawTurn < 0) {
                leftSpeed  = speed;
                rightSpeed = speed - turn;
            }
            else if (rawTurn > 0) {
                leftSpeed  = speed + turn;
                rightSpeed = speed;
            }
            else {
                rightSpeed = speed;
                leftSpeed  = speed;
            }
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