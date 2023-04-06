package com.team3181.frc2023.commands;

import com.team3181.frc2023.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SwerveAutoBalance extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private final boolean forward;
    private final Timer timer;

    public SwerveAutoBalance(boolean forward) {
        addRequirements(this.swerve);
        this.forward = forward;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        timer.restart();
        timer.stop();
    }

    @Override
    public void execute() {
        if (Math.abs(Swerve.getInstance().getPitch()) >= 0.17) {
            Swerve.getInstance().driveFieldOrientated(!forward ? -0.5 : 0.5, 0, 0);
        }
        else {
            timer.start();
            if (timer.hasElapsed(1)) {
                if (Math.abs(Swerve.getInstance().getPitch()) >= 0.1) {
                    Swerve.getInstance().driveFieldOrientated(forward ? -0.2 : 0.2, 0, 0);
                }
                else {
                    swerve.driveX();
                }
            }
            else {
                swerve.driveX();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().driveX();
    }
}