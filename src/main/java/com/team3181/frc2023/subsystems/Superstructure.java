package com.team3181.frc2023.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    enum SystemState {
        IDLE, LOW, MID, HIGH
    }

    private final static Superstructure INSTANCE = new Superstructure();

    public static Superstructure getInstance() {
        return INSTANCE;
    }

    private Superstructure() {}
}