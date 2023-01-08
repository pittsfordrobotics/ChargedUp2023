package com.team3181.frc2023.commands;


import com.team3181.frc2023.Paths;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DropClimb extends SequentialCommandGroup {
    public DropClimb(Pose2d startingPose) {
        super(
                new SwerveResetPose(startingPose),
                new InstantCommand(() -> System.out.println("Put game piece down")),
                new WaitCommand(1),
                new SwervePathing(startingPose.getY() > 2.75 ? Paths.DROP_CLIMB_HIGH : Paths.DROP_CLIMB_LOW, false)
        );
    }
}