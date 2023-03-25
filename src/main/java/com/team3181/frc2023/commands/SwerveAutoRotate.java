package com.team3181.frc2023.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team3181.frc2023.subsystems.swerve.Swerve;


public class SwerveAutoRotate extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private final Rotation2d rotation2d;

    /**
     * Creates a new SwerveAutoRotate.
     * @param swerve
     */
    public SwerveAutoRotate(Rotation2d rot) {
        addRequirements(this.swerve);
        rotation2d = rot;
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