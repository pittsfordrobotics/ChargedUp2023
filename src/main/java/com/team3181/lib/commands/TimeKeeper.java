package com.team3181.lib.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimeKeeper extends CommandBase {
    private static final Timer timer = new Timer();
    private final boolean start;

    /** @param start true for start; false for end */
    public TimeKeeper(boolean start) {
        this.start = start;
        ShuffleboardTab tab = Shuffleboard.getTab("Time");
        try {
            tab.addNumber("Auto Time", timer::get);
        } catch (Exception ignored) {}
    }

    @Override
    public void initialize() {
        if (start) {
            timer.start();
            timer.reset();
        }
        else {
            timer.stop();
            System.out.println("Autonomous took " + timer.get() + " seconds");
        }
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}