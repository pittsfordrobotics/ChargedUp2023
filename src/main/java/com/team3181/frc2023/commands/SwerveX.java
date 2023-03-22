package com.team3181.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team3181.frc2023.subsystems.swerve.Swerve;


public class SwerveX extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();

    public SwerveX() {
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        swerve.driveX();
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