package com.team3181.frc2023.commands;

import com.team3181.frc2023.subsystems.Superstructure;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SuperstructureObjectiveGlobalDemand extends CommandBase {
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Objective objective;

    public SuperstructureObjectiveGlobalDemand(Objective objective) {
        addRequirements(this.superstructure);
        this.objective = objective;
    }

    @Override
    public void initialize() {
        superstructure.objective(objective);
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