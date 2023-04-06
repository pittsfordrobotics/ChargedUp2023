package com.team3181.frc2023.commands;

import com.team3181.frc2023.Constants.SwerveConstants;
import com.team3181.frc2023.subsystems.Superstructure;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.lib.controller.BetterXboxController;
import com.team3181.lib.controller.BetterXboxController.Humans;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.littletonrobotics.junction.Logger;


public class SwerveDriveFieldXbox extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private SlewRateLimiter magLimiter;
    private SlewRateLimiter rotLimiter;
    private boolean wasRateLimiting = false;

    public SwerveDriveFieldXbox() {
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        magLimiter = new SlewRateLimiter(SwerveConstants.MAGNITUDE_RATE_LIMIT);
        rotLimiter = new SlewRateLimiter(SwerveConstants.ROTATION_RATE_LIMIT);
    }

    @Override
    public void execute() {
        //    bc we have bad controller that will drift and prolly screw up auto
        if (!DriverStation.isAutonomous()) {
            if (!Superstructure.getInstance().isStable()) {
                double xSpeed = BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getX();
                double ySpeed = BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getY();
                double rot = BetterXboxController.getController(Humans.DRIVER).getSwerveRotation();

                if (!wasRateLimiting) {
                    magLimiter.reset(Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)));
                    rotLimiter.reset(rot);
                    wasRateLimiting = true;
                }

                double mag = magLimiter.calculate(Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)));

                double xSpeedCommanded = xSpeed / SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND * mag;
                double ySpeedCommanded = ySpeed / SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND * mag;

                double rotCommanded = rotLimiter.calculate(rot);

                swerve.driveFieldOrientated(
                        swerve.isSlowMode() ? xSpeedCommanded / 2 : xSpeedCommanded,
                        swerve.isSlowMode() ? ySpeedCommanded / 2 : ySpeedCommanded,
                        swerve.isSlowMode() ? rotCommanded / 2 : rotCommanded
                );
            } else {
                wasRateLimiting = false;
                swerve.driveFieldOrientated(
                        swerve.isSlowMode() ? BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getX() / 2 : BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getX(),
                        swerve.isSlowMode() ? BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getY() / 2 : BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getY(),
                        swerve.isSlowMode() ? BetterXboxController.getController(Humans.DRIVER).getSwerveRotation() / 2 : BetterXboxController.getController(Humans.DRIVER).getSwerveRotation()
                );
            }
        }
        Logger.getInstance().recordOutput("Swerve/RateLimiting", !Superstructure.getInstance().isStable());
    }
}