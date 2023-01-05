package com.team3181.lib.controller;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.team3181.lib.controller.BetterXboxController.Hand;

public class TriggerButton extends Trigger {
    private final XboxController controller;
    private final Hand hand;

    public TriggerButton(XboxController controller, Hand hand) {
        this.controller = controller;
        this.hand = hand;
    }

    public double get() {
        return hand == Hand.LEFT ? controller.getLeftTriggerAxis() : controller.getRightTriggerAxis();
    }

    public boolean getAsBoolean(double threshold) {
        return hand == Hand.LEFT ? controller.getLeftTriggerAxis() >= threshold : controller.getRightTriggerAxis() >= threshold;
    }
}