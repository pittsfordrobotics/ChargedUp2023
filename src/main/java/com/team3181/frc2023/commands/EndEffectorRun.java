package com.team3181.frc2023.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team3181.frc2023.subsystems.endeffector.EndEffector;

public class EndEffectorRun extends CommandBase {
    private final EndEffector endEffector = EndEffector.getInstance();
    private final Timer timer = new Timer();

    public EndEffectorRun() {
        addRequirements(this.endEffector);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        endEffector.intake();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(0.2);
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.idle();
    }
}