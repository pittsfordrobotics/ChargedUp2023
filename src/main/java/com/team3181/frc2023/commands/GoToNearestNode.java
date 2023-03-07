package com.team3181.frc2023.commands;

import com.team3181.frc2023.FieldConstants;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class GoToNearestNode extends CommandBase {

    public GoToNearestNode() {
        addRequirements();
    }

    @Override
    public void initialize() {
        // make command group, add this command and another to place at node to that command group
        new SwervePathingOnTheFly(FieldConstants.AutoDrivePoints.nodeSelector(ObjectiveTracker.getInstance().getObjective().nodeRow));
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
