// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3181.frc2023;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.pathplanner.lib.PathConstraints;
import com.team3181.frc2023.subsystems.swerve.*;
import com.team3181.frc2023.subsystems.tank.TankIO;
import com.team3181.frc2023.subsystems.tank.TankIOSim;
import com.team3181.frc2023.subsystems.tank.TankIOSparkMax;
import com.team3181.lib.swerve.BetterSwerveKinematics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.HashMap;

public final class Constants {
    public static final class RobotConstants {
        public final static SwerveModuleIO FL_MODULE;
        public final static SwerveModuleIO FR_MODULE;
        public final static SwerveModuleIO BL_MODULE;
        public final static SwerveModuleIO BR_MODULE;
        public final static TankIO TANK;
        public final static GyroIO GYRO;

        public final static boolean IS_TANK = false;
        public static final boolean LOGGING_ENABLED = true;
        public static final String LOGGING_PATH = "/media/sda2/";
        public static final boolean PID_TUNER_ENABLED = false;
        public static final double LOOP_TIME_SECONDS = 0.02;

        public static final HashMap<Integer, String> SPARKMAX_HASHMAP = new HashMap<>();
        static {
            SPARKMAX_HASHMAP.put(SwerveConstants.CAN_FL_DRIVE, "Front Left Drive");
            SPARKMAX_HASHMAP.put(SwerveConstants.CAN_FL_STEER, "Front Left Steer");
            SPARKMAX_HASHMAP.put(SwerveConstants.CAN_FR_DRIVE, "Front Right Drive");
            SPARKMAX_HASHMAP.put(SwerveConstants.CAN_FR_STEER, "Front Right Steer");
            SPARKMAX_HASHMAP.put(SwerveConstants.CAN_BL_DRIVE, "Back Left Drive");
            SPARKMAX_HASHMAP.put(SwerveConstants.CAN_BL_STEER, "Back Left Steer");
            SPARKMAX_HASHMAP.put(SwerveConstants.CAN_BR_DRIVE, "Back Right Drive");
            SPARKMAX_HASHMAP.put(SwerveConstants.CAN_BR_STEER, "Back Right Steer");

            if (RobotBase.isReal()) {
                TANK = RobotConstants.IS_TANK ? new TankIOSparkMax() : new TankIO(){};
                FL_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSparkMax(SwerveConstants.CAN_FL_DRIVE, SwerveConstants.CAN_FL_STEER, SwerveConstants.FL_OFFSET);
                FR_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSparkMax(SwerveConstants.CAN_FR_DRIVE, SwerveConstants.CAN_FR_STEER, SwerveConstants.FR_OFFSET);
                BL_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSparkMax(SwerveConstants.CAN_BL_DRIVE, SwerveConstants.CAN_BL_STEER, SwerveConstants.BL_OFFSET);
                BR_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSparkMax(SwerveConstants.CAN_BR_DRIVE, SwerveConstants.CAN_BR_STEER, SwerveConstants.BR_OFFSET);
                GYRO = RobotConstants.IS_TANK ? new GyroIO(){} : new GyroIOPigeon();
            }
            else {
                TANK = RobotConstants.IS_TANK ? new TankIOSim() : new TankIO(){};
                FL_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSim();
                FR_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSim();
                BL_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSim();
                BR_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSim();
                GYRO = RobotConstants.IS_TANK ? new GyroIO(){} : new GyroIOSim();
            }
        }
    }

    public static final class SwerveConstants {
        public static final int CAN_PIGEON = 0;
        public static final Pigeon2Configuration PIGEON_CONFIG = new Pigeon2Configuration();
        static {
            PIGEON_CONFIG.EnableCompass = false;
        }

        /*
         * Swerve Module Orientation
         *    ^   FL  FR   ^
         *    |   BL  BR   |
         */
        public static final int CAN_FL_DRIVE = 5;
        public static final int CAN_FL_STEER = 6;
        public static final int CAN_FR_DRIVE = 1;
        public static final int CAN_FR_STEER = 2;
        public static final int CAN_BL_DRIVE = 7;
        public static final int CAN_BL_STEER = 8;
        public static final int CAN_BR_DRIVE = 3;
        public static final int CAN_BR_STEER = 4;

        /**
         *  Pinon    Gear Ratio    Max Speed [m/s] (approximate)
         *   12T 	   5.50:1 	      4.12
         *   13T 	   5.08:1 	      4.46
         *   14T 	   4.71:1         4.8
         */
        private enum MAX_SWERVE_GEARS {
            SLOW(12.0), MED(13.0), FAST(14.0);

            private final double gearRatio;
            private final double maxSpeed;
            MAX_SWERVE_GEARS(double pinon) {
                this.gearRatio = (45.0 * 22.0) / (pinon * 15.0);
                double WHEEL_DIAMETER_METERS = 3 * 0.0254;
                this.maxSpeed = ((5676.0 / 60.0) * WHEEL_DIAMETER_METERS * Math.PI) / gearRatio;
            }
        }

        public static final MAX_SWERVE_GEARS GEAR_CONSTANTS = MAX_SWERVE_GEARS.FAST;
        public static final double MAX_LINEAR_VELOCITY_METERS_PER_SECOND = GEAR_CONSTANTS.maxSpeed; // 1678 ran 4.5 m/s in 2022
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 10.0; // from 1678

        public static final double DRIVE_GEAR_RATIO = GEAR_CONSTANTS.gearRatio;
        public static final double STEER_GEAR_RATIO = 46.2962962963;
        public static final double X_LENGTH_METERS = Units.inchesToMeters(28);
        public static final double Y_LENGTH_METERS = Units.inchesToMeters(28);
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);

        public static final Translation2d[] MODULE_OFFSETS = {
            new Translation2d(X_LENGTH_METERS / 2, Y_LENGTH_METERS / 2), // FL
            new Translation2d(X_LENGTH_METERS / 2, -Y_LENGTH_METERS / 2), // FR
            new Translation2d(-X_LENGTH_METERS / 2, Y_LENGTH_METERS / 2), // BL
            new Translation2d(-X_LENGTH_METERS / 2, -Y_LENGTH_METERS / 2), // BR
        };
        public static final BetterSwerveKinematics DRIVE_KINEMATICS = new BetterSwerveKinematics(MODULE_OFFSETS);

        public static final Rotation2d FL_OFFSET = Rotation2d.fromRadians(0.504);
        public static final Rotation2d FR_OFFSET = Rotation2d.fromRadians(0.342);
        public static final Rotation2d BL_OFFSET = Rotation2d.fromRadians(0.98);
        public static final Rotation2d BR_OFFSET = Rotation2d.fromRadians(0.175);

        // controlling module wheel speed
        // read this later: https://github.com/Team364/BaseFalconSwerve
        public static final double MODULE_DRIVE_P = 0.04;
        public static final double MODULE_DRIVE_I = 0;
        public static final double MODULE_DRIVE_D = 0;

        // feedforward for module from SysID
        public static final double MODULE_DRIVE_S = 0;
        public static final double MODULE_DRIVE_V = 0;
        public static final double MODULE_DRIVE_A = 0;

        // controlling module position / angle
        public static final double MODULE_STEER_P = 1;
        public static final double MODULE_STEER_I = 0;
        public static final double MODULE_STEER_D = 0;
        // irl
        //
        //
        // sim
        // -0.65 for open loop
        // -0.15 closed loop
        public static final double MODULE_STEER_FF_OL = Robot.isReal() ? 0 : 0.5;
        public static final double MODULE_STEER_FF_CL = Robot.isReal() ? 0 : 0.33;
    }

    public static final class TankConstants {
        public static final int CAN_PIGEON = 0;
        public static final int CAN_RIGHT_LEADER = 3;
        public static final int CAN_RIGHT_FOLLOWER = 4;
        public static final int CAN_LEFT_LEADER = 1;
        public static final int CAN_LEFT_FOLLOWER = 2;

        public static final double WEIGHT_KILO = Units.lbsToKilograms(125);

        public static final Pigeon2Configuration PIGEON_CONFIG = new Pigeon2Configuration();
        static {
            PIGEON_CONFIG.EnableCompass = false;
        }

        public static final double GEAR_RATIO = 6.818;
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(6);

        public static final double TURNING_THROTTLE = 0.3;
        public static final double CURVE_TURNING_THROTTLE = 0.2;

        public static final double POSITION_GAIN = 2.3546;
        public static final double INTEGRAL_GAIN = 0;
        public static final double DERIVATIVE_GAIN = 0;

        public static final double STATIC_GAIN = 0.26981;
        public static final double VELOCITY_GAIN = 0.046502;
        public static final double ACCELERATION_GAIN = 0.0093369;

        public static final double TRACK_WIDTH_METERS = 0.644;

        public static final double MOI = 0.8501136363636363; // this was found in DifferentialDrivetrainSim.createKitbotSim() code

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 10;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 5;
    }

    public static final class AutoConstants {
        // PID values for trajectory follower
        public static final double LINEAR_P = 1;
        public static final double ROT_P = 5;

//        numbers from 1678
        public static final double SLOW_LINEAR_VELOCITY_METERS_PER_SECOND = 2.0;
        public static final double SLOW_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;

        public static final double MAX_LINEAR_VELOCITY_METERS_PER_SECOND = 4.0;
        public static final double MAX_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED = 4.0;

        public static final double SLOW_ANGULAR_VELOCITY_METERS_PER_SECOND = 0.8 * Math.PI;
        public static final double SLOW_ANGULAR_ACCELERATION_METERS_PER_SECOND_SQUARED = Math.pow(SLOW_ANGULAR_VELOCITY_METERS_PER_SECOND, 2);

        public static final double MAX_ANGULAR_VELOCITY_METERS_PER_SECOND = 1.2 * Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND_SQUARED = Math.pow(MAX_ANGULAR_VELOCITY_METERS_PER_SECOND, 2);

        public static final PathConstraints SLOW_SPEED = new PathConstraints(SLOW_LINEAR_VELOCITY_METERS_PER_SECOND, SLOW_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED);
        public static final PathConstraints MAX_SPEED = new PathConstraints(MAX_LINEAR_VELOCITY_METERS_PER_SECOND, MAX_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final TrapezoidProfile.Constraints MAX_ROT_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY_METERS_PER_SECOND, MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }
}