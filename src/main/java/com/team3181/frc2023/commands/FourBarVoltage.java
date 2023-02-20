package com.team3181.frc2023.commands;

import com.team3181.frc2023.subsystems.fourbar.ArmIO;
import com.team3181.frc2023.subsystems.fourbar.FourBar;
import com.team3181.lib.controller.BetterXboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class FourBarVoltage extends CommandBase {
    private final FourBar fourBar = FourBar.getInstance();
    private ArmIO io;

    public FourBarVoltage(ArmIO io) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.io = io;
        addRequirements(this.fourBar);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        io.setVoltage(BetterXboxController.getController(BetterXboxController.Humans.DRIVER).getLeftY());
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
