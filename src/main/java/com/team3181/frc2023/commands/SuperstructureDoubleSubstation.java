package com.team3181.frc2023.commands;

import com.team3181.frc2023.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SuperstructureDoubleSubstation extends CommandBase {
    private final Superstructure superstructure = Superstructure.getInstance();

    public SuperstructureDoubleSubstation() {
        addRequirements(this.superstructure);
    }

    @Override
    public void initialize() {
        superstructure.collectMid();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return superstructure.hasGamePiece();
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.home();
    }
}