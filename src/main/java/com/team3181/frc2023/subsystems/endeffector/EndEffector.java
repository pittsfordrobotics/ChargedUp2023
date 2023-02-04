package com.team3181.frc2023.subsystems.endeffector;


import com.team3181.frc2023.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.
    private final EndEffectorIO io;
    private final EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

    public WantedState mWantedState = WantedState.IDLE;
    public ActualState mActualState = ActualState.IDLE;

    private final static EndEffector INSTANCE = new EndEffector(Constants.RobotConstants.END_EFFECTOR);

    public static EndEffector getInstance() {
        return INSTANCE;
    }

    public enum WantedState {
        IDLE, INTAKING_CUBE, INTAKING_CONE, EXHAUSTING
    }
    public enum ActualState {
        IDLE, INTAKING_CUBE, INTAKING_CONE, EXHAUSTING
    }
    private EndEffector(EndEffectorIO io) {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        synchronized (EndEffector.this) {
            ActualState newState = handleAll();
            if(newState != mActualState) {
                mActualState = newState;
            }

            Logger.getInstance().processInputs("End Effector", inputs);

            switch(mActualState) {
                // would be nice if this were command based but that's annoying and periodic is easier
                case INTAKING_CONE:
                    if(inputs.positionRad < Constants.EndEffectorConstants.CONE_CLAW_POSITION) {
                        io.setVoltage(Constants.EndEffectorConstants.CONE_INTAKE_POWER);
                    }
                    break;
                case INTAKING_CUBE:
                    if(inputs.positionRad < Constants.EndEffectorConstants.CUBE_CLAW_POSITION) {
                        io.setVoltage(Constants.EndEffectorConstants.CUBE_INTAKE_POWER);
                    }
                    break;
                case EXHAUSTING:
                    if(inputs.positionRad != 0.0) {
                        io.setVoltage(Constants.EndEffectorConstants.EXHAUST_POWER);
                    }
                    break;
                default:
                case IDLE:
                    io.setVoltage(0.0);
                    // last thing command to do is call back and set state here, removes need for
                    break;
            }

            Logger.getInstance().recordOutput("End Effector/Wanted States", mWantedState.name());
            Logger.getInstance().recordOutput("End Effector/Actual States", mActualState.name());
            Logger.getInstance().recordOutput("End Effector/Angular Position", inputs.positionRad);
            Logger.getInstance().recordOutput("End Effector/Applied Voltage", inputs.appliedVolts);
            Logger.getInstance().recordOutput("End Effector/Angular Velocity", inputs.velocityRadPerSec);
        }
    }

    public ActualState handleAll() {
        switch(mWantedState) {
            case EXHAUSTING:
                return ActualState.EXHAUSTING;
            case INTAKING_CONE:
                return ActualState.INTAKING_CONE;
            case INTAKING_CUBE:
                return ActualState.INTAKING_CUBE;
            case IDLE:
            default:
                return ActualState.IDLE;
        }
    }
}

