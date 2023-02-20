package com.team3181.frc2023.subsystems;

import com.team3181.frc2023.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Superstructure extends SubsystemBase {

    enum StructureState {
        IDLE, LOW, MID, HIGH
    }

    // TODO: migrate to end effect and get game piece from there
    enum GamePiece {
        NONE, CONE, CUBE
    }

    private StructureState structureState = StructureState.IDLE;
    private StructureState wantedState = StructureState.IDLE;
    private GamePiece gamePiece = GamePiece.NONE;
    private boolean wantIntake = false;
    private boolean wantDrop = false;
    private boolean aligned = false;

    private final static Superstructure INSTANCE = new Superstructure();

    public static Superstructure getInstance() {
        return INSTANCE;
    }

    private Superstructure() {}

    @Override
    public void periodic() {
        boolean shouldAutoRetract = shouldAutoRetract();
        boolean shouldIntake = wantIntake;
        boolean shouldDrop = wantDrop;
        StructureState state = structureState;

        switch (wantedState) {
            case LOW:
                state = StructureState.LOW;
                break;
            case MID:
                state = StructureState.MID;
                break;
            case HIGH:
                state = StructureState.HIGH;
                break;
            case IDLE:
            default:
                state = StructureState.IDLE;
        };

        if (state != structureState) {
            structureState = state;
        }

        switch (structureState) {
            case LOW:
//                SET MECH TO LOW
//                SET INTAKE ON OR OFF
                break;
            case MID:
//                SET MECH TO MID
//                SET INTAKE ON OR OFF
                break;
            case HIGH:
//                SET MECH TO HIGH
//                SET INTAKE ON OR OFF
                break;
            case IDLE:
            default:
                state = StructureState.IDLE;
        }
    }

    public void structureHigh() {
        wantedState = StructureState.HIGH;
    }

    public void structureMid() {
        wantedState = StructureState.MID;
    }

    public void structureLow() {
        wantedState = StructureState.LOW;
    }

    public void collectGround() {
        wantedState = StructureState.LOW;
        wantIntake = true;
    }

    public void collectMid() {
        wantedState = StructureState.MID;
        wantIntake = true;
    }

    private boolean shouldAutoRetract() {
//        check if in center of field
        return Swerve.getInstance().getPose().getX() > 5.3 && Swerve.getInstance().getPose().getX() < 11.25;
    }
}