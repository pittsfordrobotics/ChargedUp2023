// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3181.frc2023;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import com.team3181.frc2023.subsystems.swerve.*;
import com.team3181.lib.swerve.BetterSwerveKinematics;

import java.util.HashMap;

public final class Constants {
    /*  ROBOT  */
    public static final class RobotConstants {
        public final static SwerveModuleIO FL_MODULE;
        public final static SwerveModuleIO FR_MODULE;
        public final static SwerveModuleIO BL_MODULE;
        public final static SwerveModuleIO BR_MODULE;
        public final static GyroIO GYRO;
        static {
            if (RobotBase.isReal() && RobotConstants.DEMO_MODE) {
                FL_MODULE = new SwerveModuleIOSparkMax(SwerveConstants.CAN_FL_DRIVE, SwerveConstants.CAN_FL_STEER, SwerveConstants.FL_OFFSET);
                FR_MODULE = new SwerveModuleIOSparkMax(SwerveConstants.CAN_FR_DRIVE, SwerveConstants.CAN_FR_STEER, SwerveConstants.FR_OFFSET);
                BL_MODULE = new SwerveModuleIOSparkMax(SwerveConstants.CAN_BL_DRIVE, SwerveConstants.CAN_BL_STEER, SwerveConstants.BL_OFFSET);
                BR_MODULE = new SwerveModuleIOSparkMax(SwerveConstants.CAN_BR_DRIVE, SwerveConstants.CAN_BR_STEER, SwerveConstants.BR_OFFSET);
                GYRO = new GyroIOPigeon();
            }
            else if (RobotBase.isReal()) {
                FL_MODULE = new SwerveModuleIOSparkMax(SwerveConstants.CAN_FL_DRIVE, SwerveConstants.CAN_FL_STEER, SwerveConstants.FL_OFFSET);
                FR_MODULE = new SwerveModuleIOSparkMax(SwerveConstants.CAN_FR_DRIVE, SwerveConstants.CAN_FR_STEER, SwerveConstants.FR_OFFSET);
                BL_MODULE = new SwerveModuleIOSparkMax(SwerveConstants.CAN_BL_DRIVE, SwerveConstants.CAN_BL_STEER, SwerveConstants.BL_OFFSET);
                BR_MODULE = new SwerveModuleIOSparkMax(SwerveConstants.CAN_BR_DRIVE, SwerveConstants.CAN_BR_STEER, SwerveConstants.BR_OFFSET);
                GYRO = new GyroIOPigeon();
            }
            else {
                FR_MODULE = new SwerveModuleIOSim();
                FL_MODULE = new SwerveModuleIOSim();
                BR_MODULE = new SwerveModuleIOSim();
                BL_MODULE = new SwerveModuleIOSim();
                GYRO = new GyroIOSim();
            }
        }

        public static final boolean LOGGING_ENABLED = true;
        public static final String LOGGING_PATH = "/media/sda2/";
        public static final boolean PID_TUNER_ENABLED = false;
        public static final boolean DEMO_MODE = false;
        public static final double LOOP_TIME_SECONDS = 0.02;

        public static final HashMap<Integer, String> SPARKMAX_HASHMAP = new HashMap<>();
        static {
            SPARKMAX_HASHMAP.put(1, "Front Left Drive");
            SPARKMAX_HASHMAP.put(2, "Front Left Steer");
            SPARKMAX_HASHMAP.put(3, "Front Right Drive");
            SPARKMAX_HASHMAP.put(4, "Front Right Steer");
            SPARKMAX_HASHMAP.put(5, "Back Left Drive");
            SPARKMAX_HASHMAP.put(6, "Back Left Steer");
            SPARKMAX_HASHMAP.put(7, "Back Right Drive");
            SPARKMAX_HASHMAP.put(8, "Back Right Steer");
        }
    }

    public static final class CAN {

    }

    /*  SWERVE: ALL IN METERS  */
    public static final class SwerveConstants {
        public static final int CAN_PIGEON = 0;
        public static final Pigeon2Configuration PIGEON_CONFIG = new Pigeon2Configuration();
        static {
            SwerveConstants.PIGEON_CONFIG.EnableCompass = false;
        }

        /*
         * Swerve Module Orientation
         *    ^   FL  FR   ^
         *    |   BL  BR   |
         */
        public static final int CAN_FL_DRIVE = 1;
        public static final int CAN_FL_STEER = 2;
        public static final int CAN_FR_DRIVE = 3;
        public static final int CAN_FR_STEER = 4;
        public static final int CAN_BL_DRIVE = 5;
        public static final int CAN_BL_STEER = 6;
        public static final int CAN_BR_DRIVE = 7;
        public static final int CAN_BR_STEER = 8;

        public static final int THROUGH_BORE_COUNTS_PER_REV = 8192;
        public static final double DRIVE_GEAR_RATIO = 5.08;
        public static final double STEER_GEAR_RATIO = 46.2962962963;
        public static final double X_LENGTH_METERS = Units.inchesToMeters(30);
        public static final double Y_LENGTH_METERS = Units.inchesToMeters(30);
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(3);

        public static final Translation2d[] MODULE_OFFSETS = {
            new Translation2d(X_LENGTH_METERS / 2, Y_LENGTH_METERS / 2), // FL
            new Translation2d(X_LENGTH_METERS / 2, -Y_LENGTH_METERS / 2), // FR
            new Translation2d(-X_LENGTH_METERS / 2, Y_LENGTH_METERS / 2), // BL
            new Translation2d(-X_LENGTH_METERS / 2, -Y_LENGTH_METERS / 2), // BR
        };
        public static final BetterSwerveKinematics DRIVE_KINEMATICS = new BetterSwerveKinematics(MODULE_OFFSETS);

        public static final Rotation2d FL_OFFSET = Rotation2d.fromDegrees(0);
        public static final Rotation2d FR_OFFSET = Rotation2d.fromDegrees(0);
        public static final Rotation2d BL_OFFSET = Rotation2d.fromDegrees(0);
        public static final Rotation2d BR_OFFSET = Rotation2d.fromDegrees(0);

        // controlling module wheel speed
        // read this later: https://github.com/Team364/BaseFalconSwerve
        public static final double MODULE_DRIVE_P = 0;
        public static final double MODULE_DRIVE_I = 0;
        public static final double MODULE_DRIVE_D = 0;

        // feedforward for module from SysID
        public static final double MODULE_DRIVE_S = 0;
        public static final double MODULE_DRIVE_V = 0;
        public static final double MODULE_DRIVE_A = 0;

        // controlling module position / angle
        public static final double MODULE_STEER_P = 0.01;
        public static final double MODULE_STEER_I = 0;
        public static final double MODULE_STEER_D = 0;
        // irl
        //
        //
        // sim
        // -0.65 for open loop
        // -0.15 closed loop
        public static final double MODULE_STEER_FF_OL = Robot.isReal() ? -0 : -0.65;
        public static final double MODULE_STEER_FF_CL = Robot.isReal() ? -0 : -0.15;

        /**
         *  Pinon    Gear Ratio    Max Speed (m/s)
         *   12T 	   5.50:1 	      4.12
         *   13T 	   5.08:1 	      4.46
         *   14T 	   4.71:1         4.8
         */
        public static final double MAX_LINEAR_VELOCITY_METERS_PER_SECOND = 4.46; // 1678 ran 4.5 m/s in 2022
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 10.0; // from 1678
    }

    public static final class AutoConstants {
        // PID values for trajectory follower
        public static final double LINEAR_P = 1;
        public static final double ROT_P = 5;

//        numbers from 1678
        public static final double SLOW_LINEAR_VELOCITY_METERS_PER_SECOND = 1.7;
        public static final double SLOW_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;

        public static final double MAX_LINEAR_VELOCITY_METERS_PER_SECOND = 3.0;
        public static final double MAX_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.3;

        public static final double SLOW_ANGULAR_VELOCITY_METERS_PER_SECOND = 0.8 * Math.PI;
        public static final double SLOW_ANGULAR_ACCELERATION_METERS_PER_SECOND_SQUARED = Math.pow(SLOW_ANGULAR_VELOCITY_METERS_PER_SECOND, 2);

        public static final double MAX_ANGULAR_VELOCITY_METERS_PER_SECOND = 1.2 * Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND_SQUARED = Math.pow(MAX_ANGULAR_VELOCITY_METERS_PER_SECOND, 2);

        public static final PathConstraints SLOW_SPEED = new PathConstraints(SLOW_LINEAR_VELOCITY_METERS_PER_SECOND, SLOW_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED);
        public static final PathConstraints MAX_SPEED = new PathConstraints(MAX_LINEAR_VELOCITY_METERS_PER_SECOND, MAX_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final TrapezoidProfile.Constraints MAX_ROT_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY_METERS_PER_SECOND, MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

}