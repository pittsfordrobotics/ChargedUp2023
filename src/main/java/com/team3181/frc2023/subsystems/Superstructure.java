package com.team3181.frc2023.subsystems;

import com.team3181.frc2023.Constants.SuperstructureConstants;
import com.team3181.frc2023.Constants.SuperstructureConstants.ArmPositions;
import com.team3181.frc2023.FieldConstants.AutoDrivePoints;
import com.team3181.frc2023.Robot;
import com.team3181.frc2023.subsystems.endeffector.EndEffector;
import com.team3181.frc2023.subsystems.fourbar.FourBar;
import com.team3181.frc2023.subsystems.leds.LEDs;
import com.team3181.frc2023.subsystems.leds.LEDs.LEDModes;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.lib.controller.BetterXboxController;
import com.team3181.lib.controller.BetterXboxController.Humans;
import com.team3181.lib.math.GeomUtil;
import com.team3181.lib.swerve.BetterPathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {

//    public static class SuperstructurePosition {
//        public Rotation2d elbow;
//        public Rotation2d shoulder;
//
//        public SuperstructurePosition(Rotation2d elbow, Rotation2d shoulder) {
//            this.elbow = elbow;
//            this.shoulder = shoulder;
//        }
//    }

    enum StructureState {
        IDLE, HOME, INTAKE_GROUND, INTAKE_MID, EXHAUST, OBJECTIVE, OBJECTIVE_GLOBAL, MANUAL
    }

//    public enum GamePiece {
//        CONE, CUBE, NONE
//    }

    private StructureState systemState = StructureState.HOME;
    private StructureState wantedState = StructureState.HOME;
    private double sweepGlobal = 0;
    private Objective objectiveGlobal;
    private boolean alternateLaw = false;
    private boolean autoNode = false;
    private boolean autoSubstation = false;
    private boolean demandLEDs = false;

    private final static Superstructure INSTANCE = new Superstructure();

    public static Superstructure getInstance() {
        return INSTANCE;
    }

    private Superstructure() {}

    @Override
    public void periodic() {
        boolean shouldAutoRetract = shouldAutoRetract();
        Objective objectiveLocal = ObjectiveTracker.getInstance().getObjective();
        FourBar fourBar = FourBar.getInstance();
        EndEffector endEffector = EndEffector.getInstance();
        StructureState state;

        switch (wantedState) {
            case OBJECTIVE:
                state = StructureState.OBJECTIVE;
                break;
            case OBJECTIVE_GLOBAL:
                objectiveLocal = objectiveGlobal;
                state = StructureState.OBJECTIVE;
                break;
            case INTAKE_GROUND:
                state = StructureState.INTAKE_GROUND;
//                if (endEffector.hasPiece()) {
//                    state = StructureState.HOME;
//                }
                break;
            case INTAKE_MID:
                state = StructureState.INTAKE_MID;
//                if (endEffector.hasPiece()) {
//                    state = StructureState.HOME;
//                }
                break;
            case HOME:
                state = StructureState.HOME;
                break;
            case EXHAUST:
                state = StructureState.EXHAUST;
                break;
            case MANUAL:
                state = StructureState.MANUAL;
                break;
            case IDLE:
            default:
                state = shouldAutoRetract ? StructureState.HOME : StructureState.IDLE;
                break;
        }

        // update LEDs and gamePieceLocal
        {
            LEDs leds = LEDs.getInstance();
            if (DriverStation.isDisabled()) {
                if (DriverStation.isFMSAttached()) {
                    leds.setLEDMode(LEDModes.CONNECTED_FMS);
                }
                else {
                    leds.setLEDMode(LEDModes.IDLE);
                }
            }
            else if (DriverStation.isAutonomous()) {
                leds.setLEDMode(LEDModes.RAINBOW);
            }
            else if (objectiveLocal.nodeRow == 1 || objectiveLocal.nodeRow == 4 || objectiveLocal.nodeRow == 7 || objectiveLocal.nodeLevel == NodeLevel.HYBRID) {
                if (endEffector.hasPiece()) {
                    leds.setLEDMode(LEDModes.FLASH_CUBE);
                }
//                else if (demandLEDs) {
                else {
                    leds.setLEDMode(LEDModes.CUBE);
                }
            }
            else if (objectiveLocal.nodeRow == 0 || objectiveLocal.nodeRow == 2 || objectiveLocal.nodeRow == 3 || objectiveLocal.nodeRow == 5 || objectiveLocal.nodeRow == 6 || objectiveLocal.nodeRow == 8) {
                if (endEffector.hasPiece()) {
                    leds.setLEDMode(LEDModes.FLASH_CONE);
                }
//                else if (demandLEDs) {
                else {
                    leds.setLEDMode(LEDModes.CONE);
                }
            }
            else {
                leds.setLEDMode(LEDModes.ERROR);
            }
        }
        demandLEDs = false;

        if (state != systemState) {
            if (systemState == StructureState.OBJECTIVE || systemState == StructureState.INTAKE_GROUND || systemState == StructureState.INTAKE_MID || systemState == StructureState.EXHAUST) {
                endEffector.idle();
            }
            if (systemState == StructureState.EXHAUST || systemState == StructureState.OBJECTIVE) {
                LEDs.getInstance().setLEDMode(LEDModes.IDLE);
            }
            if (state == StructureState.EXHAUST) {
                fourBar.recordDrop();
            }
            else if (state == StructureState.OBJECTIVE && objectiveLocal.nodeLevel == NodeLevel.HIGH) {
                if (objectiveLocal.nodeRow == 1 || objectiveLocal.nodeRow == 4 || objectiveLocal.nodeRow == 7) {
                    fourBar.recordHigh(new Rotation2d[]{ArmPositions.HIGH_CUBE_SHOULDER, ArmPositions.HIGH_CUBE_ELBOW});
                }
                else {
                    fourBar.recordHigh(new Rotation2d[]{ArmPositions.HIGH_CONE_SHOULDER, ArmPositions.HIGH_CONE_ELBOW});
                }
            }
            if (systemState == StructureState.HOME && !atSetpoint() && !DriverStation.isAutonomous()) {
                state = StructureState.HOME;
            }
            systemState = state;
        }

        switch (systemState) {
            case OBJECTIVE:
                LEDs.getInstance().setLEDMode(LEDModes.HAPPY);
                switch (objectiveLocal.nodeLevel) {
                    case HYBRID:
//                        if (gamePieceLocal == GamePiece.CONE || gamePieceLocal == GamePiece.CUBE) {
                            fourBar.setRotations(new Rotation2d[]{ArmPositions.HYBRID_SHOULDER, ArmPositions.HYBRID_ELBOW}, false);
//                            fourBar.setRotations(fourBar.solve(SuperstructureConstants.ArmPositions.HYBRID, false, true), false);
//                        }
                        break;
                    case MID:
                        if (objectiveLocal.nodeRow == 1 || objectiveLocal.nodeRow == 4 || objectiveLocal.nodeRow == 7) {
                            fourBar.setRotations(new Rotation2d[]{ArmPositions.MID_CUBE_SHOULDER, ArmPositions.MID_CUBE_ELBOW}, false);
                        }
                        else {
                            fourBar.setRotations(new Rotation2d[]{ArmPositions.MID_CONE_SHOULDER, ArmPositions.MID_CONE_ELBOW}, false);
                        }
//                        if (gamePieceLocal == GamePiece.CONE) {
//                            fourBar.setRotations(new Rotation2d[]{ArmPositions.MID_CONE_SHOULDER, ArmPositions.MID_CONE_ELBOW}, false);
////                            fourBar.setRotations(fourBar.solve(SuperstructureConstants.ArmPositions.MID_CONE, true, true), false);
//                        }
//                        else if (gamePieceLocal == GamePiece.CUBE) {
//                            fourBar.setRotations(new Rotation2d[]{ArmPositions.MID_CUBE_SHOULDER, ArmPositions.MID_CUBE_ELBOW}, false);
//                            fourBar.setRotations(fourBar.solve(SuperstructureConstants.ArmPositions.MID_CUBE, false, true), false);
//                        }
                        break;
                    case HIGH:
                        if (objectiveLocal.nodeRow == 1 || objectiveLocal.nodeRow == 4 || objectiveLocal.nodeRow == 7) {
                            fourBar.runHigh(new Rotation2d[]{ArmPositions.HIGH_CUBE_SHOULDER, ArmPositions.HIGH_CUBE_ELBOW});
                        }
                        else {
                            fourBar.runHigh(new Rotation2d[]{ArmPositions.HIGH_CONE_SHOULDER, ArmPositions.HIGH_CONE_ELBOW});
                        }
//                        if (gamePieceLocal == GamePiece.CONE) {
//                            fourBar.setRotations(new Rotation2d[]{ArmPositions.HIGH_CONE_SHOULDER, ArmPositions.HIGH_CONE_ELBOW}, false);
////                            fourBar.setRotations(fourBar.solve(SuperstructureConstants.ArmPositions.HIGH_CONE, true,true), false);
//                        }
//                        else if (gamePieceLocal == GamePiece.CUBE) {
//                            fourBar.setRotations(new Rotation2d[]{ArmPositions.HIGH_CUBE_SHOULDER, ArmPositions.HIGH_CUBE_ELBOW}, false);
////                            fourBar.setRotations(fourBar.solve(SuperstructureConstants.ArmPositions.HIGH_CUBE, false,true), false);
//                        }
                        break;
                }
                if (shouldAutoScore(objectiveLocal.nodeLevel) && fourBar.atSetpoint()) {
                    endEffector.exhaust();
                }
                else {
                    endEffector.idle();
                }
                break;
            case INTAKE_GROUND:
//                Translation2d pos = new Translation2d(SuperstructureConstants.ArmPositions.SWEEP_MIN.getX() + sweepLocal * (SuperstructureConstants.ArmPositions.SWEEP_MAX.getX() - SuperstructureConstants.ArmPositions.SWEEP_MIN.getX()), SuperstructureConstants.ArmPositions.SWEEP_MIN.getY());
//                fourBar.setRotations(FourBar.getInstance().solve(pos, false, true), true);
                fourBar.setRotations(new Rotation2d[]{ArmPositions.GROUND_PICKUP_SHOULDER, ArmPositions.GROUND_PICKUP_ELBOW}, false);
                if (fourBar.atSetpoint()) {
                    endEffector.intake();
                }
                else {
                    endEffector.idle();
                }
                break;
            case INTAKE_MID:
                fourBar.setRotations(new Rotation2d[]{ArmPositions.MID_PICKUP_SHOULDER, ArmPositions.MID_PICKUP_ELBOW}, false);
                if (fourBar.atSetpoint()) {
                    endEffector.intake();
                }
                else {
                    endEffector.idle();
                }
                break;
            case EXHAUST:
                fourBar.dropElbow();
                if (fourBar.atSetpoint()) {
                    fourBar.hold();
                    endEffector.exhaust();
                }
                break;
            case IDLE:
                fourBar.hold();
                endEffector.idle();
                break;
            case MANUAL:
                fourBar.setArmVoltageWithFF(0, -5 * BetterXboxController.getController(Humans.OPERATOR).getLeftY());
                fourBar.setArmVoltageWithFF(1, -5 * BetterXboxController.getController(Humans.OPERATOR).getRightY());
                endEffector.intake();
                break;
            case HOME:
            default:
                fourBar.setRotations(new Rotation2d[] {SuperstructureConstants.ArmPositions.STORAGE_SHOULDER, SuperstructureConstants.ArmPositions.STORAGE_ELBOW}, false);
                endEffector.idle();
                break;
        }

        Logger.getInstance().recordOutput("Superstructure/Wanted State", wantedState.toString());
        Logger.getInstance().recordOutput("Superstructure/System State", systemState.toString());
    }

    public void setAlternateLaw(boolean alternateLaw) {
        this.alternateLaw = alternateLaw;
    }

    public void setAutoPlace(boolean autoPlacing) {
        this.autoNode = autoPlacing;
    }

    public void setAutoSubstation(boolean autoSubstation) {
        this.autoSubstation = autoSubstation;
    }

    public void idle() {
        wantedState = StructureState.IDLE;
    }

    public void manual() {
        wantedState = StructureState.MANUAL;
    }

    public void exhaust() {
        wantedState = StructureState.EXHAUST;
    }

    public void objective() {
        wantedState = StructureState.OBJECTIVE;
    }

    public void objective(Objective objective) {
        objectiveGlobal = objective;
        wantedState = StructureState.OBJECTIVE_GLOBAL;
    }

    public void collectGround() {
        sweepGlobal = 0;
        wantedState = StructureState.INTAKE_GROUND;
    }

    public void setDemandLEDs() {
        demandLEDs = true;
    }

    /**
     *
     * @param sweep from 0-1
     */
    public void collectGround(double sweep) {
        sweepGlobal = sweep;
        wantedState = StructureState.INTAKE_GROUND;
    }

    public void collectMid() {
        wantedState = StructureState.INTAKE_MID;
    }

    public void home() {
        wantedState = StructureState.HOME;
    }

    private boolean shouldAutoRetract() {
//        check if in center of field
        return Swerve.getInstance().getPose().getX() > 5.3 && Swerve.getInstance().getPose().getX() < 11.25;
    }

    public boolean shouldAutoScore(NodeLevel nodeLevel) {
//        check if at correct pose for current node
        BetterPathPoint wantedNode = AutoDrivePoints.nodeSelector(ObjectiveTracker.getInstance().getObjective().nodeRow);
        if (nodeLevel == NodeLevel.MID) {
            wantedNode = AutoDrivePoints.adjustNodeForMid(wantedNode);
        }
        boolean rot = Math.abs(Swerve.getInstance().getPose().getRotation().getRadians() - wantedNode.getHolonomicRotation().getRadians()) < SuperstructureConstants.AUTO_SCORE_ROTATION_TOLERANCE;
        boolean position = GeomUtil.distance(Swerve.getInstance().getPose(), new Pose2d(wantedNode.getPosition(), wantedNode.getHolonomicRotation())) < SuperstructureConstants.AUTO_SCORE_POSITION_TOLERANCE;
        return rot && position && EndEffector.getInstance().hasPiece();
    }

    public boolean atSetpoint() {
        return FourBar.getInstance().atSetpoint() || Robot.isSimulation();
    }

    public boolean hasGamePiece() {
        return EndEffector.getInstance().hasPiece();
    }
}