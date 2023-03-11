package com.team3181.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team3181.frc2023.subsystems.endeffector.EndEffector;
import com.team3181.frc2023.subsystems.fourbar.FourBar;


public class ArmSweepGround extends CommandBase {
    private final EndEffector endEffector = EndEffector.getInstance();
    private final FourBar fourBar = FourBar.getInstance();

    public ArmSweepGround() {
        addRequirements(this.endEffector, this.fourBar);
    }

    @Override
    public void initialize() {
        endEffector.intake();
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return endEffector.hasPiece();
    }

    @Override
    public void end(boolean interrupted) {
        endEffector.idle();
    }
}