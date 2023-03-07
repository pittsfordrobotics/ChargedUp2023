package com.team3181.frc2023.commands;

import com.team3181.frc2023.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SuperstructureObjective extends CommandBase {
    private final Superstructure superstructure = Superstructure.getInstance();

    public SuperstructureObjective() {
        addRequirements(this.superstructure);
    }

    @Override
    public void initialize() {
        superstructure.objective();
    }

    @Override
    public boolean isFinished() {
        return superstructure.shouldGoHome();
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.home();
    }
}