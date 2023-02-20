package com.team3181.frc2023.subsystems;

import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

    enum StructureState {
        IDLE, INTAKE_GROUND, INTAKE_MID, EXPEL, MOVING, OBJECTIVE
    }

    // TODO: migrate to end effect and get game piece from there
    enum GamePiece {
        NONE, CONE, CUBE
    }

    private StructureState systemState = StructureState.IDLE;
    private StructureState wantedState = StructureState.IDLE;
    private GamePiece gamePiece = GamePiece.NONE;
    private boolean wantLunge = false;
    private boolean aligned = false;
    private Objective objective;

    private final static Superstructure INSTANCE = new Superstructure();

    public static Superstructure getInstance() {
        return INSTANCE;
    }

    private Superstructure() {
        objective = ObjectiveTracker.getInstance().getObjective();
    }

    @Override
    public void periodic() {
        boolean shouldAutoRetract = shouldAutoRetract();
        boolean shouldLunge = wantLunge;
        objective = ObjectiveTracker.getInstance().getObjective();
        StructureState state = StructureState.IDLE;

        switch (wantedState) {
            case OBJECTIVE:
                state = StructureState.OBJECTIVE;
                break;
            case IDLE:
            default:
                state = StructureState.IDLE;
        }
//        if (state != systemState) {
//            systemState = state;
//        }

//        switch (systemState) {
//            case Objective:
//                switch (objective.nodeLevel) {
//                    case HYBRID -> state = StructureState.MOVING;
//                    case MID -> state = StructureState.MIm;
//                    case HIGH -> state = StructureState.HIGH;
//                    default ->
//                }
//                break;
//            case MOVING:
//                break;
//            case IDLE:
//            default:
//                state = StructureState.IDLE;
//        }

        Logger.getInstance().recordOutput("Superstructure/Wanted State", wantedState.toString());
        Logger.getInstance().recordOutput("Superstructure/System State", systemState.toString());
    }

    public void setWantedState(StructureState state) {
        wantedState = state;
    }

    public void collectGround() {
        wantedState = StructureState.INTAKE_GROUND;
    }

    public void collectMid() {
        wantedState = StructureState.INTAKE_MID;
    }

    private boolean shouldAutoRetract() {
//        check if in center of field
        return Swerve.getInstance().getPose().getX() > 5.3 && Swerve.getInstance().getPose().getX() < 11.25;
    }
}