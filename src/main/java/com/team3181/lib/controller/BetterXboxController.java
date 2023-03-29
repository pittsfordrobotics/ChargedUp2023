package com.team3181.lib.controller;

import com.team3181.frc2023.Constants.SwerveConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.HashMap;

public class BetterXboxController extends CommandXboxController {
    private final double deadband = 0.15;

    private final BetterPOVButton DUp;
    private final BetterPOVButton DRight;
    private final BetterPOVButton DDown;
    private final BetterPOVButton DLeft;

    private static final HashMap<Humans, BetterXboxController> controllers = new HashMap<>();

    public enum Hand {
        RIGHT, LEFT
    }

    public enum Humans {
        DRIVER, OPERATOR
    }

    public BetterXboxController(int port, Humans humans) {
        super(port);
        DUp = new BetterPOVButton(getHID(), 0);
        DRight = new BetterPOVButton(getHID(), 90);
        DDown = new BetterPOVButton(getHID(), 180);
        DLeft = new BetterPOVButton(getHID(), 270);
        controllers.put(humans, this);
    }

    public static BetterXboxController getController(Humans humans) {
        return controllers.get(humans);
    }

    @Override
    public Trigger povRight() {
        return DRight;
    }

    @Override
    public Trigger povLeft() {
        return DLeft;
    }

    @Override
    public Trigger povDown() {
        return DDown;
    }

    @Override
    public Trigger povUp() {
        return DUp;
    }

    @Override
    public double getLeftY() {
        return MathUtil.applyDeadband(super.getLeftY(), deadband);
    }

    @Override
    public double getLeftX() {
        return MathUtil.applyDeadband(super.getLeftX(), deadband);
    }

    @Override
    public double getRightX() {
        return MathUtil.applyDeadband(super.getRightX(), deadband);
    }

    @Override
    public double getRightY() {
        return MathUtil.applyDeadband(super.getRightY(), deadband);
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
        getHID().setRumble(RumbleType.kLeftRumble, value);
        getHID().setRumble(RumbleType.kRightRumble, value);
    }
}