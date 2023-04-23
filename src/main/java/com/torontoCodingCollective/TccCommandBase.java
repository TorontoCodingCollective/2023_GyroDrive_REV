package com.torontoCodingCollective;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The Base TorontoCodingCollective Command Base implements command helpers to aid with logging
 */
public class TccCommandBase extends CommandBase {

    protected long commandStartTimeMillis = 0;
    private String finishReason           = null;

    public void logCommandStart() {
        logCommandStart(null);
    }

    /**
     * Log a command start
     *
     * @param commandParms string containing the parms for this command
     */
    public void logCommandStart(String commandParms) {

        finishReason           = null;
        commandStartTimeMillis = 0;

        logCommandState("STARTING", commandParms, true);

        // Set the initialize time after logging of the start message.
        commandStartTimeMillis = System.currentTimeMillis();
    }

    /**
     * Log the command end. The command start should be called before
     * calling the command end in order to start the command timer
     *
     * @param interrupted
     */
    public void logCommandEnd(boolean interrupted) {

        String state = "ENDED";

        if (interrupted) {
            state = "INTERRUPTED";
        }

        logCommandState(state, finishReason, true);
    }

    /**
     * Log a state transition in a command that has an internal state machine
     *
     * @param fromState
     * @param toState
     */
    public void logStateTransition(String fromState, String toState) {
        logStateTransition(fromState, toState, null);
    }

    /**
     * Log a state transition in a command that has an internal state machine
     *
     * @param fromState
     * @param toState
     * @param msg
     */
    public void logStateTransition(String fromState, String toState, String msg) {
        logCommandState(fromState + " -> " + toState, msg, false);
    }

    /**
     * Set the finish reason for this command. This should be set when the
     * isFinished() routine is about to return true.
     *
     * @param finishReason
     */
    public void setFinishReason(String finishReason) {
        this.finishReason = finishReason;
    }

    private void logCommandState(String state, String msg, boolean logSubsystemsFlag) {

        StringBuilder sb = new StringBuilder();

        // Log the time within the current DriverStation period
        sb.append("@").append(Math.round(DriverStation.getMatchTime() * 1000d) / 1000d).append("s ");

        // Log the command
        sb.append(this.getClass().getSimpleName());

        // Log the state (or state transition)
        sb.append(" ").append(state);

        if (msg != null) {
            sb.append(" : ").append(msg);
        }

        // Log the time since the command initialized
        if (commandStartTimeMillis != 0) {
            sb.append(" in ").append(System.currentTimeMillis() - commandStartTimeMillis).append("ms");
        }

        // Print the subsystems if required
        if (logSubsystemsFlag) {

            for (Subsystem subsystem : this.getRequirements()) {
                if (subsystem != null) {
                    sb.append("\n    ").append(subsystem.toString());
                }
            }
        }

        System.out.println(sb.toString());
    }
}
