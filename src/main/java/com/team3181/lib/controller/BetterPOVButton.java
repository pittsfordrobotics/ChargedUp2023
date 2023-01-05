package com.team3181.lib.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BetterPOVButton extends POVButton {
    private final double debounceTime = 0.1;

    public BetterPOVButton(GenericHID joystick, int angle) {
        super(joystick, angle);
    }

    @Override
    public Trigger whileTrue(Command command) {
        return super.debounce(debounceTime).whileTrue(command);
    }

    @Override
    public Trigger whileFalse(Command command) {
        return super.debounce(debounceTime).whileFalse(command);
    }

    @Override
    public Trigger toggleOnTrue(Command command) {
        return super.debounce(debounceTime).toggleOnTrue(command);
    }

    @Override
    public Trigger toggleOnFalse(Command command) {
        return super.debounce(debounceTime).toggleOnFalse(command);
    }

    @Override
    public Trigger onTrue(Command command) {
        return super.debounce(debounceTime).onTrue(command);
    }

    @Override
    public Trigger onFalse(Command command) {
        return super.debounce(debounceTime).onFalse(command);
    }
}