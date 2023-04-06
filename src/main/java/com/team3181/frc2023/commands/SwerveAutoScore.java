package com.team3181.frc2023.commands;


import com.team3181.frc2023.Constants.AutoConstants.AutoDrivePosition;
import com.team3181.frc2023.subsystems.Superstructure;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveAutoScore extends SequentialCommandGroup {
    public SwerveAutoScore() {
        super(
                new InstantCommand(() -> Superstructure.getInstance().setAutoPlace(true)),
                new SwervePathingOnTheFly(AutoDrivePosition.NODE, false),
                new SuperstructureObjective(),
                new InstantCommand(() -> Superstructure.getInstance().setAutoPlace(false))
        );
    }
}