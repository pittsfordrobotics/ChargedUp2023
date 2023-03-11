package com.team3181.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team3181.frc2023.subsystems.Superstructure;
import com.team3181.frc2023.subsystems.swerve.Swerve;


public class SuperstructureDoubleSubstation extends CommandBase {
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Swerve swerve = Swerve.getInstance();

    public SuperstructureDoubleSubstation() {
        addRequirements(this.superstructure, this.swerve);
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