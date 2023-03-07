package com.team3181.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team3181.frc2023.subsystems.Superstructure;
import com.team3181.frc2023.subsystems.swerve.Swerve;


public class SwerveDoubleSubstation extends CommandBase {
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Swerve swerve = Swerve.getInstance();

    public SwerveDoubleSubstation() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.superstructure, this.swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}