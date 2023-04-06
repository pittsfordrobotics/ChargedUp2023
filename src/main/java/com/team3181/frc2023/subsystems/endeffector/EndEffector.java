package com.team3181.frc2023.subsystems.endeffector;


import com.team3181.frc2023.Constants;
import com.team3181.frc2023.Constants.EndEffectorConstants;
import com.team3181.frc2023.Robot;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.team3181.lib.commands.DisabledInstantCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.Vector;

public class EndEffector extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.
    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    private WantedState wantedState = WantedState.IDLE;
    private WantedState lastWantedState = WantedState.IDLE;
    private ActualState actualState = ActualState.IDLE;
    private final Vector<Double> intakeCurrents = new Vector<>();
    private final int currentCycles = 20; // TODO: try increasing this
    private boolean forced = false;

    private final static EndEffector INSTANCE = new EndEffector(Constants.RobotConstants.END_EFFECTOR);
    public static EndEffector getInstance() {
        return INSTANCE;
    }

    public enum WantedState {
        IDLE, INTAKING, EXHAUSTING, STOP
    }
    public enum ActualState {
        IDLE, INTAKING, OBTAINED, EXHAUSTING, STOP
    }
    private EndEffector(EndEffectorIO io) {
        this.io = io;
        Robot.pitTab.add("EndEffector Intake", new DisabledInstantCommand(this::intake));
        Robot.pitTab.add("EndEffector Idle", new DisabledInstantCommand(this::idle));
        Robot.pitTab.add("EndEffector Exhaust", new DisabledInstantCommand(this::exhaust));
        for (int i = 0; i < currentCycles; i++) intakeCurrents.add(0.0);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        synchronized (EndEffector.this) {
            ActualState newState = handleAll();
            if(newState != actualState) {
                actualState = newState;
            }

            Logger.getInstance().processInputs("EndEffector", inputs);

            switch(actualState) {
                case INTAKING:
                    if (DriverStation.isAutonomous()) {
                        // maximize power in auto to decrease time initially
                        io.setVoltage(3);
                    } else {
                        io.setVoltage(EndEffectorConstants.INTAKE_POWER);
                    }
                    break;
                case EXHAUSTING: // may need to be 2 different values if we need to shoot cone and cube at different speeds
                    Objective objective = ObjectiveTracker.getInstance().getObjective();
                    if (objective.nodeRow == 0 || objective.nodeRow == 2 || objective.nodeRow == 3 || objective.nodeRow == 5 || objective.nodeRow == 6 || objective.nodeRow == 8) {
                        io.setVoltage(EndEffectorConstants.EXHAUST_CONE_POWER);
                    }
                    else {
                        io.setVoltage(EndEffectorConstants.EXHAUST_CUBE_POWER);
                    }
                    if (forced) {
                        io.setVoltage(-12);
                    }
                    break;
                case OBTAINED:
                case IDLE:
                default:
                    io.setVoltage(EndEffectorConstants.INTAKE_IDLE_POWER);
                    // last thing command to do is call back and set state here, removes need for
                    break;
            }
            lastWantedState = wantedState;

            Logger.getInstance().recordOutput("EndEffector/Wanted State", wantedState.toString());
            Logger.getInstance().recordOutput("EndEffector/Actual State", actualState.toString());
            Logger.getInstance().recordOutput("EndEffector/Current Array", intakeCurrents.toString());
        }
    }

    public void addGamePiece() {
        actualState = ActualState.OBTAINED;
    }

    public void setForced(boolean forced) {
        this.forced = forced;
        wantedState = WantedState.EXHAUSTING;
    }

    public void intake() {
        wantedState = WantedState.INTAKING;
    }

    public void exhaust() {
        wantedState = WantedState.EXHAUSTING;
    }

    public void idle() {
        wantedState = WantedState.IDLE;
    }

    public boolean hasPiece() {
        return actualState == ActualState.OBTAINED;
    }

    private ActualState handleAll() {
        switch(wantedState) {
            case EXHAUSTING:
                return ActualState.EXHAUSTING;
            case INTAKING:
                intakeCurrents.remove(0);
                intakeCurrents.add(inputs.currentAmps);
                return checkCurrent(lastWantedState == WantedState.IDLE);
            case IDLE:
            default:
                intakeCurrents.clear();
                for (int i = 0; i < currentCycles; i++) intakeCurrents.add(0.0);
                return (actualState != ActualState.OBTAINED) ? ActualState.IDLE : actualState;
        }
    }

    private ActualState checkCurrent(boolean newCycle) {
        if (actualState == ActualState.OBTAINED && !newCycle) {
            return actualState;
        }
        double sum = 0;
        for (int i = 0; i < currentCycles; i++) {
            sum += intakeCurrents.get(i);
        }
        double avg = sum / currentCycles;
        Logger.getInstance().recordOutput("EndEffector/Avg Current", avg);
        if (avg > 30) {
//            return ActualState.INTAKING;
            return ActualState.OBTAINED;
        }
        else {
            return ActualState.INTAKING;
        }
    }
}