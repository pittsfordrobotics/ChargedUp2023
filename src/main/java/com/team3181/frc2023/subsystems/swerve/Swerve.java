package com.team3181.frc2023.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.Constants.SwerveConstants;
import com.team3181.lib.util.*;
import com.team3181.lib.util.Alert.AlertType;
import com.team3181.lib.math.BetterMath;
import com.team3181.lib.swerve.BetterSwerveKinematics;
import com.team3181.lib.swerve.BetterSwerveModuleState;
import com.team3181.lib.swerve.SwerveOptimizer;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
    /*
     * Swerve Module Orientation
     *        FL  FR
     *        BL  BR
     */
    private final SwerveModuleIO[] moduleIO;
    private final SwerveModuleIOInputsAutoLogged[] moduleInputs = new SwerveModuleIOInputsAutoLogged[] {new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged(), new SwerveModuleIOInputsAutoLogged()};

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
    private final BetterSwerveModuleState[] lastModuleStates = new BetterSwerveModuleState[4];
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    private final SwerveDrivePoseEstimator poseEstimator;

    private Rotation2d lastRotation = new Rotation2d();
    private boolean isOpenLoop = false;

    private final Alert pigeonAlert = new Alert("Pigeon not detected! Falling back to estimated angle!", AlertType.ERROR);
    private final static Swerve INSTANCE = new Swerve(RobotConstants.FL_MODULE, RobotConstants.FR_MODULE, RobotConstants.BL_MODULE, RobotConstants.BR_MODULE, RobotConstants.GYRO);

    public static Swerve getInstance() {
        return INSTANCE;
    }

    private Swerve(SwerveModuleIO FL, SwerveModuleIO FR, SwerveModuleIO BL, SwerveModuleIO BR, GyroIO gyro) {
        moduleIO = new SwerveModuleIO[]{FL, FR, BL, BR};
        gyroIO = gyro;

        for (int i = 0; i < 4; i++) {
            moduleIO[i].updateInputs(moduleInputs[i]);
            modulePositions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
            lastModuleStates[i] = new BetterSwerveModuleState(0, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad), 0);
        }

        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.DRIVE_KINEMATICS, new Rotation2d(), modulePositions, new Pose2d());
    }

    @Override
    public void periodic() {
        SwerveModuleState[] wantedModuleStates = new SwerveModuleState[4];
        SwerveModuleState[] actualStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            moduleIO[i].updateInputs(moduleInputs[i]);
            actualStates[i] = new SwerveModuleState(moduleInputs[i].driveVelocityMetersPerSec, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
            modulePositions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, Rotation2d.fromRadians(moduleInputs[i].steerPositionRad));
            lastModuleStates[i] = DriverStation.isDisabled() ? new BetterSwerveModuleState(actualStates[i].speedMetersPerSecond, actualStates[i].angle, 0) : lastModuleStates[i]; // just to make logging look clean
            wantedModuleStates[i] = new SwerveModuleState(lastModuleStates[i].speedMetersPerSecond, Rotation2d.fromRadians(lastModuleStates[i].angle.getRadians() + lastModuleStates[i].omegaRadPerSecond * (isOpenLoop ? SwerveConstants.MODULE_STEER_FF_OL : SwerveConstants.MODULE_STEER_FF_CL) * 0.065));
        }
        chassisSpeeds = SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(actualStates);

        Logger.getInstance().processInputs("FL Swerve Module", moduleInputs[0]);
        Logger.getInstance().processInputs("FR Swerve Module", moduleInputs[1]);
        Logger.getInstance().processInputs("BL Swerve Module", moduleInputs[2]);
        Logger.getInstance().processInputs("BR Swerve Module", moduleInputs[3]);

        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Gyro", gyroInputs);

        poseEstimator.update(getRotation(), modulePositions);
        lastRotation = getRotation();

        Logger.getInstance().recordOutput("Swerve/Pose", getPose());
        Logger.getInstance().recordOutput("Swerve/Wanted States", wantedModuleStates);
        Logger.getInstance().recordOutput("Swerve/Actual States", actualStates);
        Logger.getInstance().recordOutput("Swerve/Robot Rotation Rad", getRotation().getRadians());
        Logger.getInstance().recordOutput("Swerve/Chassis Speeds X", chassisSpeeds.vxMetersPerSecond);
        Logger.getInstance().recordOutput("Swerve/Chassis Speeds Y", chassisSpeeds.vyMetersPerSecond);
        Logger.getInstance().recordOutput("Swerve/Chassis Speeds Rot", chassisSpeeds.omegaRadiansPerSecond);

        pigeonAlert.set(!gyroInputs.connected);
    }

    public void setModuleStates(BetterSwerveModuleState[] desiredModuleStates, boolean isOpenLoop) {
        this.isOpenLoop = isOpenLoop;
        BetterSwerveKinematics.desaturateWheelSpeeds(desiredModuleStates, SwerveConstants.MAX_LINEAR_VELOCITY_METERS_PER_SECOND);
        for (int i = 0; i < 4; i++) {
            desiredModuleStates[i] = SwerveOptimizer.optimize(desiredModuleStates[i], modulePositions[i].angle, Units.radiansToDegrees(lastModuleStates[i].omegaRadPerSecond * (isOpenLoop ? SwerveConstants.MODULE_STEER_FF_OL : SwerveConstants.MODULE_STEER_FF_CL) * 0.065));
            SwerveOptimizer.antiJitter(desiredModuleStates[i], lastModuleStates[i]); // anti jitter from 364 base falcon swerve
            moduleIO[i].setModuleState(desiredModuleStates[i], isOpenLoop);

            lastModuleStates[i] = desiredModuleStates[i]; // for the anti jitter code and logging
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
        setModuleStates(SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds), isOpenLoop);
    }

    public void driveFieldOrientated(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        // this uses the rotation from pose because it offsets the initial robot rotation and gyro rotation
        setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, getPose().getRotation()), true);
    }

    public void driveRobotOrientated(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        setChassisSpeeds(new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond), true);
    }

//    drives wheels at x to prevent being shoved
    public void driveX() {
        setModuleStates(new BetterSwerveModuleState[]{
                new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(315), 0),
                new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(225), 0),
                new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(45), 0),
                new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(135), 0),
        }, true);
    }

    public void setBrakeMode() {
        for (int i = 0; i < 4; i++) {
            moduleIO[i].setDriveBrakeMode(true);
            moduleIO[i].setSteerBrakeMode(true);
        }
    }

    public void setCoastMode() {
        for (int i = 0; i < 4; i++) {
            moduleIO[i].setDriveBrakeMode(false);
            moduleIO[i].setSteerBrakeMode(false);
        }
    }

    public void stopMotors() {
        moduleIO[0].stopMotors();
        moduleIO[1].stopMotors();
        moduleIO[2].stopMotors();
        moduleIO[3].stopMotors();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getRotation(), modulePositions, pose);
    }

    public void addVisionData(Pose2d pose, double time) {
//        this is recommended, but I'm not sure if it's needed
//        if (GeomUtil.distance(pose, getPose()) < 1) {
            poseEstimator.addVisionMeasurement(pose, time);
//        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public boolean isStopped() {
        boolean module0 = BetterMath.epsilonEquals(moduleInputs[0].driveVelocityMetersPerSec, 0);
        boolean module1 = BetterMath.epsilonEquals(moduleInputs[1].driveVelocityMetersPerSec, 0);
        boolean module2 = BetterMath.epsilonEquals(moduleInputs[2].driveVelocityMetersPerSec, 0);
        boolean module3 = BetterMath.epsilonEquals(moduleInputs[3].driveVelocityMetersPerSec, 0);
        return module0 && module1 && module2 && module3;
    }

    /**
     * Gets the pigeon's angle
     * @return current angle; positive = clockwise
     */
    public Rotation2d getRotation() {
        if (gyroInputs.connected) {
            return Rotation2d.fromRadians(-gyroInputs.yawPositionRad);
        }
        else {
            return Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * RobotConstants.LOOP_TIME_SECONDS + lastRotation.getRadians());
        }
    }
}