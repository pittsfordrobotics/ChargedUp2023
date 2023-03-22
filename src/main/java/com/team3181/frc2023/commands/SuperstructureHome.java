package com.team3181.frc2023.commands;

import com.team3181.frc2023.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SuperstructureHome extends CommandBase {
    private final Superstructure superstructure = Superstructure.getInstance();

    public SuperstructureHome() {
        addRequirements(this.superstructure);
    }

    @Override
    public void initialize() {
        superstructure.home();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}