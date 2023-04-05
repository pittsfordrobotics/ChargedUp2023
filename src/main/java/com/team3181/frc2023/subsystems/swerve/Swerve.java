package com.team3181.frc2023.subsystems.swerve;

import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.Constants.SwerveConstants;
import com.team3181.frc2023.Robot;
import com.team3181.lib.commands.DisabledInstantCommand;
import com.team3181.lib.math.BetterMath;
import com.team3181.lib.swerve.BetterSwerveKinematics;
import com.team3181.lib.swerve.BetterSwerveModuleState;
import com.team3181.lib.swerve.SwerveOptimizer;
import com.team3181.lib.util.Alert;
import com.team3181.lib.util.Alert.AlertType;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private ChassisSpeeds actualRobotRelativeChassisSpeeds = new ChassisSpeeds();
    private final SwerveDrivePoseEstimator poseEstimator;
//    private final PoseEstimator poseEstimator;

    private Rotation2d lastRotation = new Rotation2d();
    private boolean isOpenLoop = false;
    private boolean slowMode = false;

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
            modulePositions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, Rotation2d.fromRadians(moduleInputs[i].steerOffsetAbsolutePositionRad));
            lastModuleStates[i] = new BetterSwerveModuleState(0, Rotation2d.fromRadians(moduleInputs[i].steerOffsetAbsolutePositionRad), 0);
        }

//        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.DRIVE_KINEMATICS, getRobotRelativeAngle(), modulePositions, new Pose2d(), VecBuilder.fill(0.01, 0.01, 0.01), VecBuilder.fill(0.3, 0.3, 0.3));
        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.DRIVE_KINEMATICS, getRobotRelativeAngle(), modulePositions, new Pose2d(), VecBuilder.fill(0.003, 0.003, 0.0002), VecBuilder.fill(0.9, 0.9, 0.9));
//        poseEstimator = new PoseEstimator(VecBuilder.fill(0.003, 0.003, 0.0002));

        Robot.pitTab.add("Swerve Straight Wheels", new DisabledInstantCommand(this::forward));
        Robot.pitTab.add("Swerve Coast", new DisabledInstantCommand(this::setCoastMode));
        Robot.pitTab.add("Swerve Brake", new DisabledInstantCommand(this::setBrakeMode));
        Robot.pitTab.add("Swerve Zero Pigeon", new DisabledInstantCommand(this::zeroGyro));
    }

    @Override
    public void periodic() {
        SwerveModuleState[] wantedModuleStates = new SwerveModuleState[4];
        SwerveModuleState[] actualStates = new SwerveModuleState[4];
//        SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            moduleIO[i].updateInputs(moduleInputs[i]);
//            wheelDeltas[i] =
//                    new SwerveModulePosition(
//                            (moduleInputs[i].drivePositionMeters - modulePositions[i].distanceMeters),
//                            Rotation2d.fromRadians(moduleInputs[i].steerOffsetAbsolutePositionRad));
            actualStates[i] = new SwerveModuleState(moduleInputs[i].driveVelocityMetersPerSec, Rotation2d.fromRadians(moduleInputs[i].steerOffsetAbsolutePositionRad));
            modulePositions[i] = new SwerveModulePosition(moduleInputs[i].drivePositionMeters, Rotation2d.fromRadians(moduleInputs[i].steerOffsetAbsolutePositionRad));
            wantedModuleStates[i] = new SwerveModuleState(lastModuleStates[i].speedMetersPerSecond, Rotation2d.fromRadians(lastModuleStates[i].angle.getRadians() + lastModuleStates[i].omegaRadPerSecond * (isOpenLoop ? SwerveConstants.MODULE_STEER_FF_OL : SwerveConstants.MODULE_STEER_FF_CL) * 0.065));
        }
        actualRobotRelativeChassisSpeeds = SwerveConstants.DRIVE_KINEMATICS.toChassisSpeeds(actualStates);

//        var twist = SwerveConstants.DRIVE_KINEMATICS.toTwist2d(wheelDeltas);
//        var gyroYaw = getRobotRelativeAngle();
//        if (gyroInputs.connected) {
//            twist = new Twist2d(twist.dx, twist.dy, gyroYaw.minus(lastRotation).getRadians());
//        }
        lastRotation = getRobotRelativeAngle();
//        poseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);

        Logger.getInstance().processInputs("FL Swerve Module", moduleInputs[0]);
        Logger.getInstance().processInputs("FR Swerve Module", moduleInputs[1]);
        Logger.getInstance().processInputs("BL Swerve Module", moduleInputs[2]);
        Logger.getInstance().processInputs("BR Swerve Module", moduleInputs[3]);

        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Gyro", gyroInputs);

//        old WPILib pose estimator
        poseEstimator.update(getRobotRelativeAngle(), modulePositions);
        lastRotation = getRobotRelativeAngle();

        Logger.getInstance().recordOutput("Swerve/Pose", getPose());
        Logger.getInstance().recordOutput("Swerve/Wanted States", wantedModuleStates);
        Logger.getInstance().recordOutput("Swerve/Actual States", actualStates);
        Logger.getInstance().recordOutput("Swerve/Robot Rotation Rad", getRobotRelativeAngle().getRadians());
        Logger.getInstance().recordOutput("Swerve/Chassis Speeds X", actualRobotRelativeChassisSpeeds.vxMetersPerSecond);
        Logger.getInstance().recordOutput("Swerve/Chassis Speeds Y", actualRobotRelativeChassisSpeeds.vyMetersPerSecond);
        Logger.getInstance().recordOutput("Swerve/Chassis Speeds Rot", actualRobotRelativeChassisSpeeds.omegaRadiansPerSecond);
        Logger.getInstance().recordOutput("Swerve/Slow Mode", slowMode);

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
                new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(45), 0),
                new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(-45), 0),
                new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(315), 0),
                new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(45), 0),
        }, true);
    }

    //    drives wheels forward for debugging
    private void forward() {
        setModuleStates(new BetterSwerveModuleState[]{
                new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(0), 0),
                new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(0), 0),
                new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(0), 0),
                new BetterSwerveModuleState(0.1, Rotation2d.fromDegrees(0), 0),
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
//        poseEstimator.resetPose(pose);
        poseEstimator.resetPosition(getRobotRelativeAngle(), modulePositions, pose);
    }

    public void zeroGyro() {
        gyroIO.zeroGyro();
        Pose2d pose = new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d());
//        poseEstimator.resetPose(pose);
        poseEstimator.resetPosition(getRobotRelativeAngle(), modulePositions, pose);
    }

    public void setSlowMode(boolean slowMode) {
        this.slowMode = slowMode;
    }

    public boolean isSlowMode() {
        return slowMode;
    }

    public void addVisionData(Pose2d pose, double time, Matrix<N3, N1> vec) {
        Logger.getInstance().recordOutput("Swerve/Vision Updates", pose);
        poseEstimator.addVisionMeasurement(pose, time, vec);
    }
//

//    public void addVisionData(List<TimestampedVisionUpdate> visionData) {
//        poseEstimator.addVisionData(visionData);
//    }

//    public void addVisionData(List<TimestampedVisionUpdate> visionData) {
//        poseEstimator.addVisionData(visionData);
//    }

    public Pose2d getPose() {
//        return poseEstimator.getLatestPose();
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return actualRobotRelativeChassisSpeeds;
    }

    public boolean isStopped() {
        boolean module0 = BetterMath.epsilonEquals(moduleInputs[0].driveVelocityMetersPerSec, 0, 0.1);
        boolean module1 = BetterMath.epsilonEquals(moduleInputs[1].driveVelocityMetersPerSec, 0, 0.1);
        boolean module2 = BetterMath.epsilonEquals(moduleInputs[2].driveVelocityMetersPerSec, 0, 0.1);
        boolean module3 = BetterMath.epsilonEquals(moduleInputs[3].driveVelocityMetersPerSec, 0, 0.1);
        return module0 && module1 && module2 && module3;
    }

    /**
     * Gets the pigeon's angle
     * @return current angle; ccw+
     */
    private Rotation2d getRobotRelativeAngle() {
        if (gyroInputs.connected) {
            return Rotation2d.fromRadians(gyroInputs.yawPositionRad);
        }
        else {
            return Rotation2d.fromRadians(actualRobotRelativeChassisSpeeds.omegaRadiansPerSecond * RobotConstants.LOOP_TIME_SECONDS + lastRotation.getRadians());
        }
    }

    public double getPitch() {
        return gyroInputs.rollPositionRad;
    }
}