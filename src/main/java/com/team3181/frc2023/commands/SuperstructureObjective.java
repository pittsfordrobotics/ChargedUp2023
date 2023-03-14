package com.team3181.frc2023.commands;

import com.team3181.frc2023.subsystems.Superstructure;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SuperstructureObjective extends CommandBase {
    private final Superstructure superstructure = Superstructure.getInstance();
    private final Timer timer = new Timer();

    public SuperstructureObjective() {
        addRequirements(this.superstructure);
    }

    @Override
    public void initialize() {
        superstructure.objective();
        timer.stop();
        timer.reset();
    }

    @Override
    public void execute() {
        if (superstructure.atSetpoint()) {
            timer.start();
        }
    }

    @Override
    public boolean isFinished() {
        return !superstructure.hasGamePiece();
//        return timer.hasElapsed(SuperstructureConstants.EXHAUST_TIME);
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.home();
    }
}