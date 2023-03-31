package com.team3181.frc2023.commands;

import com.team3181.frc2023.Constants.SwerveConstants;
import com.team3181.lib.util.PIDTuner;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team3181.frc2023.subsystems.swerve.Swerve;


public class SwerveAutoRotate extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private final Rotation2d rotation2d;
    private final PIDController pidController = new PIDController(SwerveConstants.AUTO_ROTATE_P, SwerveConstants.AUTO_ROTATE_I, SwerveConstants.AUTO_ROTATE_D);
    private final PIDTuner pidTuner = new PIDTuner("SwerveAutoRotate", pidController);

    /**
     * Creates a new SwerveAutoRotate.
     * @param rot wanted {@link Rotation2d} relative to field
     */
    public SwerveAutoRotate(Rotation2d rot) {
        addRequirements(this.swerve);
        rotation2d = rot;
        pidController.setTolerance(SwerveConstants.AUTO_ROTATE_TOLERANCE);
    }

    @Override
    public void initialize() {
        pidController.setSetpoint(rotation2d.getRadians());
    }

    @Override
    public void execute() {
        swerve.driveFieldOrientated(0, 0, pidController.calculate(swerve.getPose().getRotation().getRadians()));
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {

    }
}