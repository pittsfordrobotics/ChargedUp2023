package com.team3181.frc2023.subsystems;

import com.team3181.frc2023.subsystems.endeffector.EndEffector;
import com.team3181.frc2023.subsystems.fourbar.FourBar;
import com.team3181.frc2023.subsystems.leds.LEDs;
import com.team3181.frc2023.subsystems.leds.LEDs.LEDModes;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

    public static class SuperstructurePosition {
        public Rotation2d elbow;
        public Rotation2d shoulder;

        public SuperstructurePosition(Rotation2d elbow, Rotation2d shoulder) {
            this.elbow = elbow;
            this.shoulder = shoulder;
        }
    }

    enum StructureState {
        IDLE, INTAKE_GROUND, INTAKE_MID, EXPEL, MOVING, OBJECTIVE
    }

    private StructureState systemState = StructureState.IDLE;
    private StructureState wantedState = StructureState.IDLE;
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
        FourBar fourBar = FourBar.getInstance();
        EndEffector endEffector = EndEffector.getInstance();
        StructureState state = StructureState.IDLE;

        switch (wantedState) {
            case OBJECTIVE:
                state = StructureState.OBJECTIVE;
                break;
            case INTAKE_GROUND:
                state = StructureState.INTAKE_GROUND;
                break;
            case INTAKE_MID:
                state = StructureState.INTAKE_MID;
                break;
            case EXPEL:
                state = StructureState.EXPEL;
                break;
            case MOVING:
                state = StructureState.MOVING;
                break;
            case IDLE:
            default:
                state = StructureState.IDLE;
                break;
        }
        if (state != systemState) {
            systemState = state;
        }

        switch (systemState) {
            case OBJECTIVE:
                switch (objective.nodeLevel) {
                    case HYBRID:
                    case MID:
                    case HIGH:
                    default:
                }
                break;
            case MOVING:
                break;
            case IDLE:
            default:
                state = StructureState.IDLE;
        }

        // update LEDs
        {
            LEDs leds = LEDs.getInstance();
            if (DriverStation.isDisabled()) {
                leds.setLEDMode(LEDModes.IDLE);
            } else if (DriverStation.isAutonomous()) {
                leds.setLEDMode(LEDModes.RAINBOW);
            } else if (objective.nodeRow == 1 || objective.nodeRow == 4 || objective.nodeRow == 7 || objective.nodeLevel == NodeLevel.HYBRID) {
                if (endEffector.hasPiece()) {
                    leds.setLEDMode(LEDModes.CUBE);
                } else {
                    leds.setLEDMode(LEDModes.FLASH_CUBE);
                }
            } else if (objective.nodeRow == 0 || objective.nodeRow == 3 || objective.nodeRow == 6) {
                if (endEffector.hasPiece()) {
                    leds.setLEDMode(LEDModes.CONE);
                } else {
                    leds.setLEDMode(LEDModes.FLASH_CONE);
                }
            } else {
                if (endEffector.hasPiece()) {
                    leds.setLEDMode(LEDModes.CONE);
                } else {
                    leds.setLEDMode(LEDModes.FLASH_CONE);
                }
            }
        }

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