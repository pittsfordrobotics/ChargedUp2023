package com.team3181.lib.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TimedDeadlineGroup extends ParallelDeadlineGroup {
    public TimedDeadlineGroup(double time, Command... commands) {
        super(
                new WaitCommand(time),
                commands
        );
    }
}