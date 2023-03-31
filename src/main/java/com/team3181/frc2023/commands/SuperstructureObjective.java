package com.team3181.frc2023.commands;

import com.team3181.frc2023.Constants.SuperstructureConstants;
import com.team3181.frc2023.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SuperstructureObjective extends CommandBase {
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Timer timer = new Timer();

    public SuperstructureObjective() {
        addRequirements(this.superstructure);
    }

    @Override
    public void initialize() {
        superstructure.objective();
        timer.restart();
        timer.stop();
    }

    @Override
    public void execute() {
        if (superstructure.atSetpoint()) {
            timer.start();
            superstructure.exhaust();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(SuperstructureConstants.EXHAUST_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.home();
    }
}