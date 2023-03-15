package com.team3181.frc2023.commands;

import com.team3181.frc2023.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SwerveAutoBalance extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private final boolean forward;

    public SwerveAutoBalance(boolean forward) {
        addRequirements(this.swerve);
        this.forward = forward;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (!forward) {
            Swerve.getInstance().driveFieldOrientated(-0.5, 0, 0);
        }
        else {
            Swerve.getInstance().driveFieldOrientated(0.5, 0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        if (!forward) {
            return Math.abs(Swerve.getInstance().getPitch().getRadians()) < 0.005;
        }
        else {
            return Math.abs(Swerve.getInstance().getPitch().getRadians()) > -0.005;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().driveX();
    }
}