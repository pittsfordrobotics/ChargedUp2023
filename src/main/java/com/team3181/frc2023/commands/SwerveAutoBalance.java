package com.team3181.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team3181.frc2023.subsystems.swerve.Swerve;


public class SwerveAutoBalance extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private boolean onRamp;
    private boolean overBalanced;

    public SwerveAutoBalance() {
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        onRamp = false;
        overBalanced = false;
    }

    @Override
    public void execute() {
        if (onRamp && Math.abs(Swerve.getInstance().getPitch().getRadians()) < 0.1) {
            overBalanced = true;
        }
        if (!overBalanced) {
            Swerve.getInstance().driveFieldOrientated(-0.5, 0, 0);
        }
        else {
            Swerve.getInstance().driveFieldOrientated(0.1, 0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(Swerve.getInstance().getPitch().getRadians()) > 0.1) {
            onRamp = true;
        }
        return onRamp && Math.abs(Swerve.getInstance().getPitch().getRadians()) < 0.01;
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().driveX();
    }
}