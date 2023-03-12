// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team3181.frc2023.subsystems.fourbar;

import com.team3181.frc2023.Constants.EndEffectorConstants;
import com.team3181.frc2023.Constants.FourBarConstants;
import com.team3181.frc2023.subsystems.fourbar.ArmConfig.JointConfig;
import com.team3181.frc2023.subsystems.fourbar.ArmConfig.MotorConfig;
import edu.wpi.first.math.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Converts between the system state and motor voltages for a double jointed arm.
 *
 * <p>https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060
 *
 * <p>https://www.chiefdelphi.com/t/double-jointed-arm-physics-control-simulator/424307
 */
public class ArmDynamics {
    private static final double g = 9.80665;
    private final ArmConfig.JointConfig shoulder;
    private final ArmConfig.JointConfig elbow;

    private final static ArmDynamics INSTANCE = new ArmDynamics();

    public static ArmDynamics getInstance() {
        return INSTANCE;
    }

    private ArmDynamics() {
        shoulder = new JointConfig(
                FourBarConstants.SHOULDER_MASS,
                FourBarConstants.SHOULDER_LENGTH,
                FourBarConstants.SHOULDER_MOI + FourBarConstants.SHOULDER_MASS * Math.pow(FourBarConstants.SHOULDER_PIVOT_LENGTH, 2),
                FourBarConstants.SHOULDER_CG_RADIUS,
                FourBarConstants.SHOULDER_MIN.getRadians(),
                FourBarConstants.SHOULDER_MIN.getRadians(),
                new MotorConfig(DCMotor.getNEO(2), FourBarConstants.SHOULDER_GEAR_RATIO * FourBarConstants.CHAIN_RATIO)
        );

        // Combine elbow and wrist constants
        var elbowCgRadius = (FourBarConstants.ELBOW_CG_RADIUS * FourBarConstants.ELBOW_MASS
                + (FourBarConstants.ELBOW_LENGTH + EndEffectorConstants.CG_RADIUS) * EndEffectorConstants.MASS)
                / (FourBarConstants.ELBOW_MASS + EndEffectorConstants.MASS);
        var elbowMoi =
                FourBarConstants.ELBOW_MASS * Math.pow(FourBarConstants.ELBOW_CG_RADIUS - elbowCgRadius, 2.0)
                        + EndEffectorConstants.MASS
                        * Math.pow(
                        FourBarConstants.ELBOW_LENGTH + EndEffectorConstants.CG_RADIUS - elbowCgRadius, 2.0);
        elbow = new JointConfig(
                FourBarConstants.ELBOW_MASS + EndEffectorConstants.MASS,
                FourBarConstants.ELBOW_FULL_LENGTH + EndEffectorConstants.LENGTH,
                elbowMoi,
                elbowCgRadius,
                FourBarConstants.ELBOW_MIN.getRadians(),
                FourBarConstants.ELBOW_MAX.getRadians(),
                new MotorConfig(DCMotor.getNEO(1), FourBarConstants.ELBOW_GEAR_RATIO * FourBarConstants.CHAIN_RATIO));
    }

    /** Calculates the joint voltages based on the joint positions (feedforward). */
    public Vector<N2> feedforward(Vector<N2> position) {
        return feedforward(position, VecBuilder.fill(0.0, 0.0), VecBuilder.fill(0.0, 0.0));
    }

    /**
     * Calculates the joint voltages based on the full joint states as a matrix (feedforward). The
     * rows represent each joint and the columns represent position, velocity, and acceleration.
     */
    public Vector<N2> feedforward(Matrix<N2, N3> state) {
        return feedforward(
                new Vector<>(state.extractColumnVector(0)),
                new Vector<>(state.extractColumnVector(1)),
                new Vector<>(state.extractColumnVector(2)));
    }

    /** Calculates the joint voltages based on the full joint states as vectors (feedforward). */
    public Vector<N2> feedforward(Vector<N2> position, Vector<N2> velocity, Vector<N2> acceleration) {
        var torque =
                M(position)
                        .times(acceleration)
                        .plus(C(position, velocity).times(velocity))
                        .plus(Tg(position));
        return VecBuilder.fill(
                shoulder.motor().physics().getVoltage(torque.get(0, 0), velocity.get(0, 0)) / 500.0,
                elbow.motor().physics().getVoltage(torque.get(1, 0), velocity.get(1, 0)) / (400.0 * (position.get(1, 0) > 1.5 ? 1 : -1)));
    }

    /**
     * Adjusts the simulated state of the arm based on applied voltages.
     *
     * @param state The current state of the arm as (position_0, position_1, velocity_0, velocity_1)
     * @param voltage The applied voltage of each joint.
     * @param dt The step length in seconds.
     * @return The new state of the arm as (position_0, position_1, velocity_0, velocity_1)
     */
    public Vector<N4> simulate(Vector<N4> state, Vector<N2> voltage, double dt) {
        return new Vector<>(
                NumericalIntegration.rkdp(
                        (Matrix<N4, N1> x, Matrix<N2, N1> u) -> {
                            // x = current state, u = voltages, return = state derivatives

                            // Get vectors from state
                            var position = VecBuilder.fill(x.get(0, 0), x.get(1, 0));
                            var velocity = VecBuilder.fill(x.get(2, 0), x.get(3, 0));

                            // Calculate torque
                            var shoulderTorque =
                                    shoulder
                                            .motor()
                                            .physics()
                                            .getTorque(
                                                    shoulder.motor().physics().getCurrent(velocity.get(0, 0), u.get(0, 0)));
                            var elbowTorque =
                                    elbow
                                            .motor()
                                            .physics()
                                            .getTorque(
                                                    elbow.motor().physics().getCurrent(velocity.get(1, 0), u.get(1, 0)));
                            var torque = VecBuilder.fill(shoulderTorque, elbowTorque);

                            // Apply limits
                            if (position.get(0, 0) < shoulder.minAngle()) {
                                position.set(0, 0, shoulder.minAngle());
                                if (velocity.get(0, 0) < 0.0) {
                                    velocity.set(0, 0, 0.0);
                                }
                                if (torque.get(0, 0) < 0.0) {
                                    torque.set(0, 0, 0.0);
                                }
                            }
                            if (position.get(0, 0) > shoulder.maxAngle()) {
                                position.set(0, 0, shoulder.maxAngle());
                                if (velocity.get(0, 0) > 0.0) {
                                    velocity.set(0, 0, 0.0);
                                }
                                if (torque.get(0, 0) > 0.0) {
                                    torque.set(0, 0, 0.0);
                                }
                            }
                            if (position.get(1, 0) < elbow.minAngle()) {
                                position.set(1, 0, elbow.minAngle());
                                if (velocity.get(1, 0) < 0.0) {
                                    velocity.set(1, 0, 0.0);
                                }
                                if (torque.get(1, 0) < 0.0) {
                                    torque.set(1, 0, 0.0);
                                }
                            }
                            if (position.get(1, 0) > elbow.maxAngle()) {
                                position.set(1, 0, elbow.maxAngle());
                                if (velocity.get(1, 0) > 0.0) {
                                    velocity.set(1, 0, 0.0);
                                }
                                if (torque.get(1, 0) > 0.0) {
                                    torque.set(1, 0, 0.0);
                                }
                            }

                            // Calculate acceleration
                            var acceleration =
                                    M(position)
                                            .inv()
                                            .times(
                                                    torque.minus(C(position, velocity).times(velocity)).minus(Tg(position)));

                            // Return state vector
                            return new MatBuilder<>(Nat.N4(), Nat.N1())
                                    .fill(
                                            velocity.get(0, 0),
                                            velocity.get(1, 0),
                                            acceleration.get(0, 0),
                                            acceleration.get(1, 0));
                        },
                        state,
                        voltage,
                        dt));
    }

    private Matrix<N2, N2> M(Vector<N2> position) {
        var M = new Matrix<>(N2.instance, N2.instance);
        M.set(
                0,
                0,
                shoulder.mass() * Math.pow(shoulder.cgRadius(), 2.0)
                        + elbow.mass() * (Math.pow(shoulder.length(), 2.0) + Math.pow(elbow.cgRadius(), 2.0))
                        + shoulder.moi()
                        + elbow.moi()
                        + 2
                        * elbow.mass()
                        * shoulder.length()
                        * elbow.cgRadius()
                        * Math.cos(position.get(1, 0)));
        M.set(
                1,
                0,
                elbow.mass() * Math.pow(elbow.cgRadius(), 2.0)
                        + elbow.moi()
                        + elbow.mass() * shoulder.length() * elbow.cgRadius() * Math.cos(position.get(1, 0)));
        M.set(
                0,
                1,
                elbow.mass() * Math.pow(elbow.cgRadius(), 2.0)
                        + elbow.moi()
                        + elbow.mass() * shoulder.length() * elbow.cgRadius() * Math.cos(position.get(1, 0)));
        M.set(1, 1, elbow.mass() * Math.pow(elbow.cgRadius(), 2.0) + elbow.moi());
        return M;
    }

    private Matrix<N2, N2> C(Vector<N2> position, Vector<N2> velocity) {
        var C = new Matrix<>(N2.instance, N2.instance);
        C.set(
                0,
                0,
                -elbow.mass()
                        * shoulder.length()
                        * elbow.cgRadius()
                        * Math.sin(position.get(1, 0))
                        * velocity.get(1, 0));
        C.set(
                1,
                0,
                elbow.mass()
                        * shoulder.length()
                        * elbow.cgRadius()
                        * Math.sin(position.get(1, 0))
                        * velocity.get(0, 0));
        C.set(
                0,
                1,
                -elbow.mass()
                        * shoulder.length()
                        * elbow.cgRadius()
                        * Math.sin(position.get(1, 0))
                        * (velocity.get(0, 0) + velocity.get(1, 0)));
        return C;
    }

    private Matrix<N2, N1> Tg(Vector<N2> position) {
        var Tg = new Matrix<>(N2.instance, N1.instance);
        Tg.set(
                0,
                0,
                (shoulder.mass() * shoulder.cgRadius() + elbow.mass() * shoulder.length())
                        * g
                        * Math.cos(position.get(0, 0))
                        + elbow.mass()
                        * elbow.cgRadius()
                        * g
                        * Math.cos(position.get(0, 0) + position.get(1, 0)));
        Tg.set(
                1,
                0,
                elbow.mass() * elbow.cgRadius() * g * Math.cos(position.get(0, 0) + position.get(1, 0)));
        return Tg;
    }
}