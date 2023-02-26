// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3181.frc2023;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.pathplanner.lib.PathConstraints;
import com.team3181.frc2023.subsystems.endeffector.EndEffectorIO;
import com.team3181.frc2023.subsystems.endeffector.EndEffectorIOSparkMax;
import com.team3181.frc2023.subsystems.fourbar.ArmIO;
import com.team3181.frc2023.subsystems.fourbar.ArmIOElbowSparkMax;
import com.team3181.frc2023.subsystems.fourbar.ArmIOShoulderSparkMax;
import com.team3181.frc2023.subsystems.leds.LEDStripIO;
import com.team3181.frc2023.subsystems.leds.LEDStripIORio;
import com.team3181.frc2023.subsystems.objectivetracker.NodeSelectorIO;
import com.team3181.frc2023.subsystems.objectivetracker.NodeSelectorIOServer;
import com.team3181.frc2023.subsystems.swerve.*;
import com.team3181.frc2023.subsystems.tank.TankIO;
import com.team3181.frc2023.subsystems.tank.TankIOSim;
import com.team3181.frc2023.subsystems.tank.TankIOSparkMax;
import com.team3181.frc2023.subsystems.vision.VisionIO;
import com.team3181.frc2023.subsystems.vision.VisionIOLimelight;
import com.team3181.frc2023.subsystems.vision.VisionIOSim;
import com.team3181.lib.swerve.BetterSwerveKinematics;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.HashMap;

public final class Constants {
    public static final class RobotConstants {
        public final static EndEffectorIO END_EFFECTOR;
        public final static SwerveModuleIO FL_MODULE;
        public final static SwerveModuleIO FR_MODULE;
        public final static SwerveModuleIO BL_MODULE;
        public final static SwerveModuleIO BR_MODULE;
        public final static GyroIO GYRO;
        public final static TankIO TANK;
        public final static VisionIO VISION;
        public final static LEDStripIO LEDS;
        public final static ArmIO SHOULDER;
        public final static ArmIO ELBOW;
        public final static NodeSelectorIO NODE_SELECTOR;

        public final static boolean IS_TANK = false;
        public static final boolean LOGGING_ENABLED = true;
        public static final boolean REPLAY_ENABLED = false;
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
            SPARKMAX_HASHMAP.put(EndEffectorConstants.CAN_MAIN, "Intake");
            SPARKMAX_HASHMAP.put(FourBarConstants.CAN_SHOULDER_MASTER, "Shoulder Master");
            SPARKMAX_HASHMAP.put(FourBarConstants.CAN_SHOULDER_FOLLOWER, "Shoulder Follower");
            SPARKMAX_HASHMAP.put(FourBarConstants.CAN_ELBOW, "Elbow");

            if (RobotBase.isReal()) {
                SHOULDER = new ArmIOShoulderSparkMax();
                ELBOW = new ArmIOElbowSparkMax();
                TANK = RobotConstants.IS_TANK ? new TankIOSparkMax() : new TankIO(){};
                FL_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSparkMax(SwerveConstants.CAN_FL_DRIVE, SwerveConstants.CAN_FL_STEER, SwerveConstants.FL_OFFSET);
                FR_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSparkMax(SwerveConstants.CAN_FR_DRIVE, SwerveConstants.CAN_FR_STEER, SwerveConstants.FR_OFFSET);
                BL_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSparkMax(SwerveConstants.CAN_BL_DRIVE, SwerveConstants.CAN_BL_STEER, SwerveConstants.BL_OFFSET);
                BR_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSparkMax(SwerveConstants.CAN_BR_DRIVE, SwerveConstants.CAN_BR_STEER, SwerveConstants.BR_OFFSET);
                GYRO = RobotConstants.IS_TANK ? new GyroIO(){} : new GyroIOPigeon();
                VISION = RobotConstants.IS_TANK ? new VisionIO() {} : new VisionIOLimelight();
                LEDS = RobotConstants.IS_TANK ? new LEDStripIO(){} : new LEDStripIORio(LEDConstants.PWM_PORT, LEDConstants.NUMBER);
                END_EFFECTOR = new EndEffectorIOSparkMax();
            }
            else {
                SHOULDER = new ArmIO(){};
                ELBOW = new ArmIO(){};
                TANK = RobotConstants.IS_TANK ? new TankIOSim() : new TankIO(){};
                FL_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSim();
                FR_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSim();
                BL_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSim();
                BR_MODULE = RobotConstants.IS_TANK ? new SwerveModuleIO(){} : new SwerveModuleIOSim();
                GYRO = RobotConstants.IS_TANK ? new GyroIO(){} : new GyroIOSim();
                VISION = new VisionIOSim();
                LEDS = new LEDStripIO() {};
                END_EFFECTOR = new EndEffectorIO() {};
            }
            NODE_SELECTOR = new NodeSelectorIOServer();
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
        public static final int CAN_BL_DRIVE = 5; // RF
        public static final int CAN_BL_STEER = 6; // RF
        public static final int CAN_BR_DRIVE = 1; // LF
        public static final int CAN_BR_STEER = 2; // LF
        public static final int CAN_FL_DRIVE = 7; // RR
        public static final int CAN_FL_STEER = 8; // RR
        public static final int CAN_FR_DRIVE = 3; // LR
        public static final int CAN_FR_STEER = 4; // LR

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

        public static final Rotation2d FL_OFFSET = Rotation2d.fromRadians(0.1715);
        public static final Rotation2d FR_OFFSET = Rotation2d.fromRadians(5.167658 + Math.PI);
        public static final Rotation2d BL_OFFSET = Rotation2d.fromRadians(3.11452428);
        public static final Rotation2d BR_OFFSET = Rotation2d.fromRadians(4.138);

        // controlling module wheel speed
        // read this later: https://github.com/Team364/BaseFalconSwerve
        public static final double MODULE_DRIVE_P = 0.0001;
        public static final double MODULE_DRIVE_I = 0;
        public static final double MODULE_DRIVE_D = 0;

        // feedforward for module from SysID
        public static final double MODULE_DRIVE_S = 0;
        public static final double MODULE_DRIVE_V = 0;
        public static final double MODULE_DRIVE_A = 0;
        public static final double MODULE_DRIVE_FF = 1 / MAX_LINEAR_VELOCITY_METERS_PER_SECOND;

        // controlling module position / angle
        public static final double MODULE_STEER_P = 1.5;
        public static final double MODULE_STEER_I = 0;
        public static final double MODULE_STEER_D = 0;
        // irl
        //
        //
        // sim
        // -0.65 for open loop
        // -0.15 closed loop
        public static final double MODULE_STEER_FF_OL = Robot.isReal() ? 0.4 : 0.5;
        public static final double MODULE_STEER_FF_CL = Robot.isReal() ? 0.9 : 0.33;

        public static final double DIRECTION_RATE_LIMIT = 15; // radians per second
        public static final double MAGNITUDE_RATE_LIMIT = 10.7; // percent per second (1 = 100%)
        public static final double ROTATION_RATE_LIMIT = 16.0; // percent per second (1 = 100%)
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
        public static final double LINEAR_P = 1.5;
        public static final double ROT_P = 5;

        //        numbers from 1678
        public static final double SLOW_LINEAR_VELOCITY_METERS_PER_SECOND = 2.0;
        public static final double SLOW_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED = 2.0;

        public static final double MAX_LINEAR_VELOCITY_METERS_PER_SECOND = 4.8;
        public static final double MAX_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED = 6.0;

        public static final double SLOW_ANGULAR_VELOCITY_METERS_PER_SECOND = 0.8 * Math.PI;
        public static final double SLOW_ANGULAR_ACCELERATION_METERS_PER_SECOND_SQUARED = Math.pow(SLOW_ANGULAR_VELOCITY_METERS_PER_SECOND, 2);

        public static final double MAX_ANGULAR_VELOCITY_METERS_PER_SECOND = 1.2 * Math.PI;
        public static final double MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND_SQUARED = Math.pow(MAX_ANGULAR_VELOCITY_METERS_PER_SECOND, 2);

        public static final PathConstraints SLOW_SPEED = new PathConstraints(SLOW_LINEAR_VELOCITY_METERS_PER_SECOND, SLOW_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED);
        public static final PathConstraints MAX_SPEED = new PathConstraints(MAX_LINEAR_VELOCITY_METERS_PER_SECOND, MAX_LINEAR_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final TrapezoidProfile.Constraints MAX_ROT_CONSTRAINTS = new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY_METERS_PER_SECOND, MAX_ANGULAR_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

    public static final class FourBarConstants {
        public static final int CAN_SHOULDER_MASTER = 10;
        public static final int CAN_SHOULDER_FOLLOWER = 11;
        public static final int CAN_ELBOW = 12;

        public static final double SHOULDER_GEAR_RATIO = 5 * 4 * 4;
        public static final double ELBOW_GEAR_RATIO = 5 * 5 * 3;
        public static final double CHAIN_RATIO = 1.0 / 3.0;

        public static final double PID_CLAMP_VOLTAGE = 5;

        public static final double ELBOW_P = -10.0;
        public static final double ELBOW_I = 0.0;
        public static final double ELBOW_D = 0.0;

        public static final double SHOULDER_P = 10.0;
        public static final double SHOULDER_I = 0.0;
        public static final double SHOULDER_D = 0.0;

        public static final double PID_TOLERANCE = 0.05;

        public static final Rotation2d SHOULDER_ABSOLUTE_OFFSET = Rotation2d.fromRadians(0.627);
        public static final Rotation2d ELBOW_ABSOLUTE_OFFSET = Rotation2d.fromRadians(5.46);
        public static final Rotation2d SHOULDER_MATH_OFFSET = Rotation2d.fromRadians(-1.0077); // zero needs to be at shoulder parallel to ground
        public static final Rotation2d ELBOW_MATH_OFFSET = Rotation2d.fromRadians(-3.52); // zero is in line with shoulder

        public static final Rotation2d ELBOW_MIN = Rotation2d.fromDegrees(-140);
        public static final Rotation2d ELBOW_MAX = Rotation2d.fromDegrees(140);

        public static final Rotation2d SHOULDER_MIN = Rotation2d.fromRadians(-1.3); // est -1.22 rad
        public static final Rotation2d SHOULDER_MAX = Rotation2d.fromDegrees(70); // est 170 deg

        public static double WHEEL_TO_CHASSIS = Units.inchesToMeters(4.75);
        public static double CHASSIS_TO_ARM = Units.inchesToMeters(28.9);

        public static double SHOULDER_FULL_LENGTH = Units.inchesToMeters(41.5);
        public static double SHOULDER_PIVOT_LENGTH = Units.inchesToMeters(13);
        public static double SHOULDER_LENGTH = SHOULDER_FULL_LENGTH - SHOULDER_PIVOT_LENGTH;

        public static double ELBOW_FULL_LENGTH = Units.inchesToMeters(28);
        public static double ELBOW_PIVOT_LENGTH = Units.inchesToMeters(1.5 + (1.0/16.0));
        public static double ELBOW_LENGTH = ELBOW_FULL_LENGTH - ELBOW_PIVOT_LENGTH;

        public static double SHOULDER_MOI = 0.22952; // kg m^2
        public static double ELBOW_MOI = 1.417864996; // kg m^2

        public static double SHOULDER_CG = Units.inchesToMeters(20.74886);
        public static double ELBOW_CG = Units.inchesToMeters(13.998858);

        public static double SHOULDER_CG_RADIUS = SHOULDER_CG - SHOULDER_PIVOT_LENGTH;
        public static double ELBOW_CG_RADIUS = ELBOW_CG - ELBOW_PIVOT_LENGTH;

        public static double SHOULDER_MASS = Units.lbsToKilograms(5.45192);
        public static double ELBOW_MASS = Units.lbsToKilograms(8.745 - 6); // elbow + end effector

        public static double SHOULDER_JOINT_POSITION_X = Units.inchesToMeters(-14); // zero is front of chassis at ground level
        public static double SHOULDER_JOINT_POSITION_Y = WHEEL_TO_CHASSIS + CHASSIS_TO_ARM;

        public static double BUMPER_THICKNESS = Units.inchesToMeters(4);

        public static final class ArmPositions {
            public static Rotation2d STORAGE_SHOULDER = Rotation2d.fromRadians(-1.22);
            public static Rotation2d STORAGE_ELBOW = Rotation2d.fromRadians(1.6);

            public static Rotation2d GROUND_PICKUP_SHOULDER = Rotation2d.fromRadians(-1.15);
            public static Rotation2d GROUND_PICKUP_ELBOW = Rotation2d.fromRadians(-0.1);

            public static Translation2d SWEEP_MIN = new Translation2d(0.66, 0.3);
            public static Translation2d SWEEP_MAX = new Translation2d(1.05, 0.3);

            public static Translation2d MID_INTAKE = new Translation2d(1, 1.1);

            public static Translation2d HYBRID = new Translation2d(0, 0);

            public static Translation2d MID_CONE = new Translation2d(0.6 + BUMPER_THICKNESS, 0.9);
            public static Translation2d HIGH_CONE = new Translation2d(1 + BUMPER_THICKNESS, 1.25);

            public static Translation2d MID_CUBE = new Translation2d(0.35 + BUMPER_THICKNESS, 0.68);
            public static Translation2d HIGH_CUBE = new Translation2d(0.9 + BUMPER_THICKNESS, 1);
        }
    }

    public static final class EndEffectorConstants {
        public static final double EXHAUST_POWER = -12.0;
        public static final double INTAKE_POWER = 12.0;
        public static final double GEARING = 4 * 3 * 3;

        public static final int CAN_MAIN = 9;
        public static double MASS = Units.lbsToKilograms(6 +  5.2/ 16.0);
        public static double CG_RADIUS = Units.inchesToMeters(1.5);

        public static double LENGTH = Units.inchesToMeters(7.15);
        public static double CUBE_CENTER = Units.inchesToMeters(6.275);
        public static double CONE_CENTER = Units.inchesToMeters(3.6);
    }

    public static final class LEDConstants {
        public static final int PWM_PORT = 0;
        public static final int NUMBER = 20;
    }
}