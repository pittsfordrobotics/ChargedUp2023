package com.team3181.frc2023.commands;

import com.team3181.frc2023.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team3181.frc2023.subsystems.endeffector.EndEffector;
import com.team3181.frc2023.subsystems.fourbar.FourBar;


public class SuperstructurePosition extends CommandBase {
    private final Superstructure superstructure = Superstructure.getInstance();

    public SuperstructurePosition() {
        addRequirements(this.superstructure);
    }

    @Override
    public void initialize() {
        superstructure.objective();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return ;
    }

    @Override
    public void end(boolean interrupted) {
        superstructure.home();
    }
}