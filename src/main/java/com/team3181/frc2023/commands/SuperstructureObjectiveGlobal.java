package com.team3181.frc2023.commands;

import com.team3181.frc2023.Constants.SuperstructureConstants;
import com.team3181.frc2023.subsystems.Superstructure;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SuperstructureObjectiveGlobal extends CommandBase {
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Timer timer = new Timer();
    private final Objective objective;

    public SuperstructureObjectiveGlobal(Objective objective) {
        addRequirements(this.superstructure);
        this.objective = objective;
    }

    @Override
    public void initialize() {
        superstructure.objective(objective);
        timer.stop();
        timer.reset();
    }

    @Override
    public void execute() {
        if (superstructure.shouldAutoScore()) {
            timer.start();
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