package com.team3181.frc2023.subsystems.endeffector;


import com.team3181.frc2023.Constants;
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
    private ActualState actualState = ActualState.IDLE;
    private Vector<Double> intakeCurrents = new Vector<>();
    private final int currentCycles = 20;

    private final static EndEffector INSTANCE = new EndEffector(Constants.RobotConstants.END_EFFECTOR);
    public static EndEffector getInstance() {
        return INSTANCE;
    }

    public enum WantedState {
        IDLE, INTAKING, EXHAUSTING
    }
    public enum ActualState {
        IDLE, INTAKING, CONE_OBTAINED, CUBE_OBTAINED, EXHAUSTING
    }
    private EndEffector(EndEffectorIO io) {
        this.io = io;
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

            Logger.getInstance().processInputs("End Effector", inputs);

            switch(actualState) {
                // would be nice if this were command based but that's annoying and periodic is easier
                case INTAKING:
                    io.setVoltage(Constants.EndEffectorConstants.INTAKE_POWER);
                    break;
                case EXHAUSTING: // may need to be 2 different values if we need to shoot cone and cube at different speeds
                    io.setVoltage(Constants.EndEffectorConstants.EXHAUST_POWER);
                    break;
                case CONE_OBTAINED:
                case CUBE_OBTAINED:
                case IDLE:
                default:
                    io.setVoltage(0.0);
                    // last thing command to do is call back and set state here, removes need for
                    break;
            }

            Logger.getInstance().recordOutput("End Effector/Wanted State", wantedState.toString());
            Logger.getInstance().recordOutput("End Effector/Actual State", actualState.toString());
            Logger.getInstance().recordOutput("End Effector/Current Array", intakeCurrents.toString());
        }
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

    private ActualState handleAll() {
        switch(wantedState) {
            case EXHAUSTING:
                return ActualState.EXHAUSTING;
            case INTAKING:
                intakeCurrents.remove(0);
                intakeCurrents.add(inputs.currentAmps);
                return checkCurrent();
            case IDLE:
            default:
                intakeCurrents.clear();
                for (int i = 0; i < currentCycles; i++) intakeCurrents.add(0.0);
                return ((actualState != ActualState.CONE_OBTAINED) && (actualState != ActualState.CUBE_OBTAINED)) ? ActualState.IDLE : actualState;
        }
    }

    private ActualState checkCurrent() {
        if (actualState == ActualState.CONE_OBTAINED || actualState == ActualState.CUBE_OBTAINED) {
            return actualState;
        }
        double sum = 0;
        for (int i = 0; i < currentCycles; i++) {
            sum += intakeCurrents.get(i);
        }
        double avg = sum / currentCycles;
        Logger.getInstance().recordOutput("End Effector/Avg Current", avg);
        System.out.println(avg);
        if (avg > 11) {
            return ActualState.CUBE_OBTAINED;
        }
        else if (avg > 10) {
            return ActualState.CONE_OBTAINED;
        }
        else {
            return ActualState.INTAKING;
        }
    }
}