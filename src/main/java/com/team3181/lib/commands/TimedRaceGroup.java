package com.team3181.lib.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TimedRaceGroup extends ParallelRaceGroup {
    public TimedRaceGroup(double time, Command... commands) {
        super(commands);
        addCommands(new WaitCommand(time));
    }
}