package com.team3181.frc2023.subsystems.tank;

import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.Constants.TankConstants;
import com.team3181.lib.util.Alert;
import com.team3181.lib.util.Alert.AlertType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Tank extends SubsystemBase {
    private final TankIO io;
    private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();

    private double throttle;
    private Pose2d pose = new Pose2d(0, 0, Rotation2d.fromDegrees(getAngle()));
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getAngle()), inputs.leftPositionMeters, inputs.rightPositionMeters, pose);
    private DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(0, 0);

    private double lastLeftPositionMeters = 0.0;
    private double lastRightPositionMeters = 0.0;
    private boolean lastGyroConnected = false;
    private Rotation2d lastGyroRotation = new Rotation2d();
    private static double quickStopAccumulator = 0.0;

    private final Alert pigeonAlert = new Alert("Pigeon not detected! Many functions of the robot will FAIL!", AlertType.ERROR);

    private static final Tank INSTANCE = new Tank(RobotConstants.TANK);
    public static Tank getInstance() {
        return INSTANCE;
    }

    private Tank(TankIO io) {
        this.io = io;

        setThrottle(0.7);

        ShuffleboardTab driveTab = Shuffleboard.getTab("Tank");
        driveTab.addNumber("Pigeon", this::getAngle);
        driveTab.addNumber("Throttle", this::getThrottle);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Tank", inputs);

        pigeonAlert.set(!inputs.gyroConnected);

        Logger.getInstance().recordOutput("Tank/Throttle", throttle);

        pose = odometry.update(
                Rotation2d.fromDegrees(getAngle()),
                inputs.leftPositionMeters,
                inputs.rightPositionMeters);

        Rotation2d currentGyroRotation =
                new Rotation2d(inputs.gyroYawPositionRad * -1);
        double leftPositionMetersDelta =
                inputs.leftPositionMeters - lastLeftPositionMeters;
        double rightPositionMetersDelta =
                inputs.rightPositionMeters - lastRightPositionMeters;
        double avgPositionMetersDelta =
                (leftPositionMetersDelta + rightPositionMetersDelta) / 2.0;
        Rotation2d gyroRotationDelta =
                (inputs.gyroConnected && !lastGyroConnected) ? new Rotation2d()
                        : currentGyroRotation.minus(lastGyroRotation);

//        if (inputs.gyroConnected) {
//            RobotState.getInstance().addDriveData(Timer.getFPGATimestamp(), new Twist2d(
//                    avgPositionMetersDelta, 0.0, gyroRotationDelta.getRadians()));
//        } else {
//            RobotState.getInstance().addDriveData(Timer.getFPGATimestamp(),
//                    new Twist2d(avgPositionMetersDelta, 0.0,
//                            (rightPositionMetersDelta - leftPositionMetersDelta)
//                                    / TankConstants.TRACK_WIDTH_METERS));
//        }

        lastLeftPositionMeters = inputs.leftPositionMeters;
        lastRightPositionMeters = inputs.rightPositionMeters;
        lastGyroConnected = inputs.gyroConnected;
        lastGyroRotation = currentGyroRotation;


        Logger.getInstance().recordOutput("Tank/Pose",
                new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()});

        wheelSpeeds = new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
    }

    // this should be removed in favor of setVolts for future
    @Deprecated(forRemoval = true)
    public void rotate(double rotation) {
        WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(0, rotation, false);
        io.set(speeds.left * 0.6, speeds.right * 0.6);
    }

    public void driveCurve(double speed, double rotation) {
         WheelSpeeds speeds = Tank.curvatureDrive(speed, rotation / 1.5, Math.abs(speed) < 0.1, true);
//        these two are irelevent
//        if (MathUtil.applyDeadband(speed,0.1) == 0) {
//            speedRateLimiter.reset(0);
//        }
//        if (MathUtil.applyDeadband(rotation,0.1) == 0) {
//            rotRateLimiter.reset(0);
//        }
//        double limitedSpeed = speedRateLimiter.calculate(speed);
//        double limitedRot = rotRateLimiter.calculate(rotation * TankConstants.TURNING_THROTTLE);
//
////        use this when not getting attacked
//        WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(MathUtil.applyDeadband(limitedSpeed,0.1), MathUtil.applyDeadband(limitedRot,0.1), Math.abs(speed) < 0.1);

//        WheelSpeeds speeds = DifferentialDrive.curvatureDriveIK(MathUtil.applyDeadband(speed,0.05), MathUtil.applyDeadband(rotation,0.05), Math.abs(speed) < 0.1);
//        io.set(speeds.left * throttle, speeds.right * throttle);

        if (Math.abs(speed) < 0.1) {
            io.set(speeds.left, speeds.right);
        }
        else {
            io.set(speeds.left * throttle, speeds.right * throttle);
        }
    }

    public void setVolts(double left, double right) {
        io.setVoltage(left, right);
    }

    public void setThrottle(double throttle) {
        this.throttle = throttle;
    }

    public double getThrottle() {
        return throttle;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(getAngle()), inputs.leftPositionMeters, inputs.rightPositionMeters, pose);
//        RobotState.getInstance().resetPose(pose);
        resetEncoders();
    }

    public void resetEncoders() {
        io.resetEncoders();
    }

    public double getLeftVelocity() {
        return inputs.leftVelocityMetersPerSec;
    }

    public double getRightVelocity() {
        return inputs.rightVelocityMetersPerSec;
    }

    public double getAverageVelocity() { return (getLeftVelocity() + getRightVelocity()) / 2; }

    public PIDController getLeftController() {
        return new PIDController(TankConstants.POSITION_GAIN, TankConstants.INTEGRAL_GAIN, TankConstants.DERIVATIVE_GAIN);
    }

    public PIDController getRightController() {
        return new PIDController(TankConstants.POSITION_GAIN, TankConstants.INTEGRAL_GAIN, TankConstants.DERIVATIVE_GAIN);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return wheelSpeeds;
    }

    public void coastMode() {
        io.setBrakeMode(false);
    }

    public void brakeMode() {
        io.setBrakeMode(true);
    }

    public Pose2d getPose() {
        return pose;
//        TODO: use this sucks (don't fix this) (its magic from other teams)
//        return RobotState.getInstance().getLatestPose()
    }

    /**
     * Gets the pigeon's angle
     * @return current angle; positive = ccw
     */
    public double getAngle() {
        return -Units.radiansToDegrees(inputs.gyroYawPositionRad);
    }

    public static WheelSpeeds curvatureDrive(double throttle, double turn, boolean isQuickTurn, boolean squaredInputs) {
        throttle = handleDeadzone(throttle, 0.1);
        turn = handleDeadzone(turn, 0.1);

        double overPower;
        double angularPower;

        if (squaredInputs) {
            // square the inputs (while preserving the sign) to increase fine control
            // while permitting full power
            if (throttle >= 0.0) {
                throttle = throttle * throttle;
            } else {
                throttle = -(throttle * throttle);
            }
        }

        if (isQuickTurn) {
            if (Math.abs(throttle) < 0.2) {
                double alpha = 0.1;
                quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * limit(turn, 1.0) * 2;
            }
            overPower = 1.0;
            angularPower = turn;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * turn * 1.0 - quickStopAccumulator;
            if (quickStopAccumulator > 1) {
                quickStopAccumulator -= 1;
            } else if (quickStopAccumulator < -1) {
                quickStopAccumulator += 1;
            } else {
                quickStopAccumulator = 0.0;
            }
        }

        double rightPwm = throttle - angularPower;
        double leftPwm = throttle + angularPower;
        if (leftPwm > 1.0) {
            rightPwm -= overPower * (leftPwm - 1.0);
            leftPwm = 1.0;
        } else if (rightPwm > 1.0) {
            leftPwm -= overPower * (rightPwm - 1.0);
            rightPwm = 1.0;
        } else if (leftPwm < -1.0) {
            rightPwm += overPower * (-1.0 - leftPwm);
            leftPwm = -1.0;
        } else if (rightPwm < -1.0) {
            leftPwm += overPower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }

        return new WheelSpeeds(leftPwm, rightPwm);
    }

    /**
     * Handles a deadzone
     *
     * @param value    The value to handle
     * @param deadzone The deadzone
     * @return The handled value
     */
    protected static double handleDeadzone(double value, double deadzone) {
        return (Math.abs(value) > Math.abs(deadzone)) ? limit(value, 1.0) : 0.0;
    }

    /**
     * Limits a number between a given range
     *
     * @param value The value to limit
     * @param max   The absolute value of the maximum value
     * @return The limited value
     */
    protected static double limit(double value, double max) {
        if (value > max) {
            return max;
        }
        if (value < -max) {
            return -max;
        }
        return value;
    }

}