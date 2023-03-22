package com.team3181.frc2023.commands;


import com.team3181.frc2023.Paths;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoCollectAndGo extends SequentialCommandGroup {
    public AutoCollectAndGo() {
        super(
            new SwervePathing(Paths.PIECE_AUTO_HIGH_NOTGROUP, true)
        );
    }
}