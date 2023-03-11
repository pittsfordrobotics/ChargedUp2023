package com.team3181.frc2023.commands;

import com.team3181.frc2023.Constants.SwerveConstants;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.team3181.frc2023.subsystems.vision.Vision;


public class AprilAuto extends CommandBase {
    private final Vision vision = Vision.getInstance();
    private final Swerve swerve = Swerve.getInstance();

    public AprilAuto() {
        addRequirements(this.vision, this.swerve);
    }

    @Override
    public void execute() {
        swerve.resetPose(vision.getPose());
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}