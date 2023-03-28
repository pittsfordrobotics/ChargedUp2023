package com.team3181.frc2023.commands;

import com.team3181.frc2023.subsystems.Superstructure;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SuperstructureObjectiveGlobalWait extends CommandBase {
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Timer timer = new Timer();
    private final Objective objective;

    public SuperstructureObjectiveGlobalWait(Objective objective) {
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