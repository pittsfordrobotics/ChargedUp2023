package com.team3181.lib.controller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import com.team3181.frc2023.Constants.SwerveConstants;

import java.util.HashMap;

public class BetterXboxController extends XboxController {
    private final double deadband = 0.15;

    public final JoystickButton A;
    public final JoystickButton B;
    public final JoystickButton X;
    public final JoystickButton Y;
    public final JoystickButton LB;
    public final JoystickButton RB;
    public final BetterPOVButton DUp;
    public final BetterPOVButton DRight;
    public final BetterPOVButton DDown;
    public final BetterPOVButton DLeft;
    public final TriggerButton LT;
    public final TriggerButton RT;
    public final JoystickButton Start;
    public final JoystickButton Back;

    private static final HashMap<Humans, BetterXboxController> controllers = new HashMap<>();

    public enum Hand {
        RIGHT, LEFT
    }

    public enum Humans {
        DRIVER, OPERATOR
    }

    public BetterXboxController(int port, Humans humans) {
        super(port);
        A = new JoystickButton(this, XboxController.Button.kA.value);
        B = new JoystickButton(this, XboxController.Button.kB.value);
        X = new JoystickButton(this, XboxController.Button.kX.value);
        Y = new JoystickButton(this, XboxController.Button.kY.value);
        LB = new JoystickButton(this, XboxController.Button.kLeftBumper.value);
        RB = new JoystickButton(this, XboxController.Button.kRightBumper.value);
        DUp = new BetterPOVButton(this, 0);
        DRight = new BetterPOVButton(this, 90);
        DDown = new BetterPOVButton(this, 180);
        DLeft = new BetterPOVButton(this, 270);
        LT = new TriggerButton(this, Hand.LEFT);
        RT = new TriggerButton(this, Hand.RIGHT);
        Back = new JoystickButton(this, XboxController.Button.kBack.value);
        Start = new JoystickButton(this, XboxController.Button.kStart.value);
        controllers.put(humans, this);
    }

    public static BetterXboxController getController(Humans humans) {
        return controllers.get(humans);
    }

    // from 1678 2022 control board
    public Translation2d getSwerveTranslation() {
        double forwardAxis = -getLeftY();
        double strafeAxis = -getLeftX();

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < deadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.getX(), tAxes.getY());
            Translation2d deadband_vector = new Translation2d(deadband, deadband_direction);

            double scaled_x = tAxes.getX() - (deadband_vector.getX()) / (1 - deadband_vector.getX());
            double scaled_y = tAxes.getY() - (deadband_vector.getY()) / (1 - deadband_vector.getY());
            return new Translation2d(scaled_x * SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND, scaled_y * SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND);
        }
    }

    // from 1678 2022 control board
    public double getSwerveRotation() {
        double rotAxis = -getRightX();

        if (Math.abs(rotAxis) < deadband) {
            return 0.0;
        } else {
            return SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * (rotAxis - (Math.signum(rotAxis) * deadband)) / (1 - deadband);
        }
    }

    /** @param value between 0 and 1 */
    public void setRumble(double value) {
        setRumble(RumbleType.kLeftRumble, value);
        setRumble(RumbleType.kRightRumble, value);
    }
}