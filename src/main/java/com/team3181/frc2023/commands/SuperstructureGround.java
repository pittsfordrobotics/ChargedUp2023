package com.team3181.frc2023.commands;

import com.team3181.frc2023.Constants.SuperstructureConstants;
import com.team3181.frc2023.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SuperstructureGround extends CommandBase {
    private final Superstructure superstructure = Superstructure.getInstance();

    public SuperstructureGround() {
        addRequirements(this.superstructure);
    }

    @Override
    public void initialize() {
        superstructure.collectGround();
    }

    @Override
    public boolean isFinished() {
        return superstructure.hasGamePiece();
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.home();
    }
}