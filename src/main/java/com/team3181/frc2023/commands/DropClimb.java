package com.team3181.frc2023.commands;


import com.team3181.lib.swerve.BetterPathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DropClimb extends SequentialCommandGroup {
    public DropClimb(Pose2d startingPose) {
        super(
                new SwerveResetPose(startingPose),
                new InstantCommand(() -> System.out.println("Put game piece down")),
                new WaitCommand(1),
                new SwervePathingOnTheFly(new BetterPathPoint(new Translation2d(), new Rotation2d()))
        );
    }
}