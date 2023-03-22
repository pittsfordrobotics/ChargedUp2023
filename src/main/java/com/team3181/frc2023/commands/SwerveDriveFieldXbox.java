package com.team3181.frc2023.commands;

import com.team3181.frc2023.Constants.SwerveConstants;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.lib.controller.BetterXboxController;
import com.team3181.lib.controller.BetterXboxController.Humans;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SwerveDriveFieldXbox extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private SlewRateLimiter m_magLimiter;
    private SlewRateLimiter m_rotLimiter;
    private double m_prevTime = WPIUtilJNI.now() * 1e-6;
    private double m_currentTranslationDir = 0.0;
    private double m_currentTranslationMag = 0.0;
    private final boolean rateLimit = true;

    public SwerveDriveFieldXbox() {
        addRequirements(this.swerve);
    }

    @Override
    public void initialize() {
        m_magLimiter = new SlewRateLimiter(SwerveConstants.MAGNITUDE_RATE_LIMIT);
        m_rotLimiter = new SlewRateLimiter(SwerveConstants.ROTATION_RATE_LIMIT);
    }

    @Override
    public void execute() {
        //    bc we have bad controller that will drift and prolly screw up auto
        if (!DriverStation.isAutonomous()) {
//            if (rateLimit) {
//                double xSpeed = BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getX();
//                double ySpeed = BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getY();
//                double rot = BetterXboxController.getController(Humans.DRIVER).getSwerveRotation();
//
//                    // Convert XY to polar for rate limiting
//                    double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
//                    double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
//
//                    // Calculate the direction slew rate based on an estimate of the lateral acceleration
//                    double directionSlewRate;
//                    if (m_currentTranslationMag != 0.0) {
//                        directionSlewRate = Math.abs(SwerveConstants.DIRECTION_RATE_LIMIT / m_currentTranslationMag);
//                    } else {
//                        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
//                    }
//
//                    double currentTime = WPIUtilJNI.now() * 1e-6;
//                    double elapsedTime = currentTime - m_prevTime;
//                    double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
//                    if (angleDif < 0.45 * Math.PI) {
//                        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
//                        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
//                    } else if (angleDif > 0.85 * Math.PI) {
//                        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
//                            // keep currentTranslationDir unchanged
//                            m_currentTranslationMag = m_magLimiter.calculate(0.0);
//                        } else {
//                            m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
//                            m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
//                        }
//                    } else {
//                        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
//                        m_currentTranslationMag = m_magLimiter.calculate(0.0);
//                    }
//                    m_prevTime = currentTime;
//
//                    double xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
//                    double ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
//                    double rotCommanded = m_rotLimiter.calculate(rot);
//
//                    swerve.driveFieldOrientated(
//                            xSpeedCommanded,
//                            ySpeedCommanded,
//                            rotCommanded
//                    );
//                }
            if (rateLimit) {
                double xSpeed = BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getX();
                double ySpeed = BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getY();
                double rot = BetterXboxController.getController(Humans.DRIVER).getSwerveRotation();

                double mag = m_magLimiter.calculate(Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2)));

                double xSpeedCommanded = xSpeed / SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND * mag;
                double ySpeedCommanded = ySpeed / SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND * mag;
                double rotCommanded = m_rotLimiter.calculate(rot);

                swerve.driveFieldOrientated(
                        xSpeedCommanded,
                        ySpeedCommanded,
                        rotCommanded
                );
            }
            else {
                    swerve.driveFieldOrientated(
                            BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getX(),
                            BetterXboxController.getController(Humans.DRIVER).getSwerveTranslation().getY(),
                            BetterXboxController.getController(Humans.DRIVER).getSwerveRotation()
                    );
                }
            }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}