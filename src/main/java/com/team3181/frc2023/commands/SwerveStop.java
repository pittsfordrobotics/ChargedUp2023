package com.team3181.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team3181.frc2023.subsystems.swerve.Swerve;


public class SwerveStop extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();

    public SwerveStop() {
        addRequirements(this.swerve);
    }

    @Override
    public void execute() {
        swerve.stopMotors();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}