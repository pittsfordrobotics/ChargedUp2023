package com.team3181.frc2023;

import com.team3181.frc2023.Constants.SwerveConstants;
import com.team3181.lib.swerve.BetterPathPoint;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.List;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise.
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall. 
 */

// from 6328
public final class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(651.25); // 16.54
    public static final double fieldWidth = Units.inchesToMeters(315.5);
    public static final double tapeWidth = Units.inchesToMeters(2.0);

    /**
     * ALL DEFINED IN BLUE, YOU MUST USE PATHPOINTFLIPPER!!!!!
     */
    public static final class AutoDrivePoints {
        public static final BetterPathPoint BOTTOM_NODE = new BetterPathPoint(new Translation2d(SwerveConstants.BUMPER_WIDTH + Grids.outerX + SwerveConstants.X_LENGTH_METERS / 2, Grids.nodeFirstY), Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(-180));

        // exit is closer to the inside of the arena and farther from the nodes
        public static final BetterPathPoint COMMUNITY_TOP_EXIT = new BetterPathPoint(new Translation2d(Community.midX + SwerveConstants.X_LENGTH_METERS, Community.leftY - Units.inchesToMeters(65) / 2), Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(-180));
        public static final BetterPathPoint COMMUNITY_BOTTOM_EXIT = new BetterPathPoint(new Translation2d(Community.outerX + SwerveConstants.X_LENGTH_METERS, 0.5), Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(-180));

        // inner is toward the outside of the arena closer to the nodes
        public static final BetterPathPoint COMMUNITY_TOP_INNER = new BetterPathPoint(new Translation2d(Grids.outerX + 1.12, Community.leftY - 0.8), Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(-180));
        public static final BetterPathPoint COMMUNITY_BOTTOM_INNER = new BetterPathPoint(new Translation2d(Grids.outerX + 1.12, 0.8), Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(-180));

        public static final BetterPathPoint COMMUNITY_MID_INNER = new BetterPathPoint(new Translation2d(2.25, 2.78), Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(-180));

        public static final BetterPathPoint LOADING_STATION_TOP_EXIT = new BetterPathPoint(new Translation2d(0.5 * LoadingZone.midX + LoadingZone.outerX * 0.5, LoadingZone.midY + Units.inchesToMeters(50.5) / 2), Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(-180));
        public static final BetterPathPoint LOADING_STATION_BOTTOM_EXIT = new BetterPathPoint(new Translation2d(0.5 * LoadingZone.midX + LoadingZone.outerX * 0.5, LoadingZone.midY - Units.inchesToMeters(50.5)/ 2), Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(-180));

        public static final BetterPathPoint LOADING_STATION_TOP_INNER = new BetterPathPoint(new Translation2d(-SwerveConstants.BUMPER_WIDTH + LoadingZone.doubleSubstationX - SwerveConstants.X_LENGTH_METERS / 2 - 0.2 - 0.5, LoadingZone.midY + Units.inchesToMeters(50.5) / 2), Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(0));
        public static final BetterPathPoint LOADING_STATION_BOTTOM_INNER = new BetterPathPoint(new Translation2d(-SwerveConstants.BUMPER_WIDTH + LoadingZone.doubleSubstationX - SwerveConstants.X_LENGTH_METERS / 2 - 0.2 - 0.5, LoadingZone.midY - Units.inchesToMeters(50.5) / 2), Rotation2d.fromDegrees(-180), Rotation2d.fromDegrees(0));

        /**
         * @param startingPoint point 1
         * @param endingPoint point 2
         * @return Point 1 with updated heading to point at ending point
         */
        public static BetterPathPoint updateHeading(BetterPathPoint startingPoint, BetterPathPoint endingPoint) {
            double deltaX = endingPoint.getPosition().getX() - startingPoint.getPosition().getX();
            double deltaY = endingPoint.getPosition().getY() - startingPoint.getPosition().getY();
            Rotation2d updatedHeading;
            if (deltaY == 0) {
                updatedHeading = Rotation2d.fromDegrees(180 * deltaX > 0 ? 0 : 1);
            }
            else if(deltaY > 0){
                updatedHeading = Rotation2d.fromRadians(Math.atan((deltaY)/(deltaX)));
            } else {
                updatedHeading = Rotation2d.fromRadians(Math.atan((deltaX)/(deltaY)));
            }
            if (deltaX > 0 && deltaY > 0) {
                updatedHeading = updatedHeading.times(-1);
            }
            else if (deltaX < 0 && deltaY > 0) {
                updatedHeading = updatedHeading.plus(Rotation2d.fromDegrees(180));
            }
            else if (deltaX < 0 && deltaY < 0) {
                updatedHeading = updatedHeading.times(-1);
                updatedHeading = updatedHeading.plus(Rotation2d.fromDegrees(-90));
            }
            else if (deltaX > 0 && deltaY < 0) {
                updatedHeading = updatedHeading.plus(Rotation2d.fromDegrees(90));
            }
            else if (deltaX == 0) {
                updatedHeading = updatedHeading.plus(Rotation2d.fromDegrees(90 * deltaY > 0 ? 1 : -1));
            }
            return new BetterPathPoint(startingPoint.getPosition(), updatedHeading, startingPoint.getHolonomicRotation());
        }

        /**
         * @param node Select a node (0-8): starting at the bottom
         * @return PathPoint to the selected node on the Blue Alliance
         */
        public static BetterPathPoint nodeSelector(int node) {
            return new BetterPathPoint(new Translation2d(BOTTOM_NODE.getPosition().getX(), BOTTOM_NODE.getPosition().getY() + Grids.nodeSeparationY * (node)), BOTTOM_NODE.getHeading(), BOTTOM_NODE.getHolonomicRotation());
        }

        public static BetterPathPoint adjustNodeForHyrbid(BetterPathPoint point) {
            return new BetterPathPoint(new Translation2d(point.getPosition().getX() + 0.33006, point.getPosition().getY()), point.getHeading(), point.getHolonomicRotation());
        }

        /**
         * @param betterPathPoint path point to be flipped
         * @return changes the heading as if you were leaving the community
         */
        public static BetterPathPoint leavingCommunity(BetterPathPoint betterPathPoint) {
            return new BetterPathPoint(betterPathPoint.getPosition(), betterPathPoint.getHeading().plus(Rotation2d.fromDegrees(180)), betterPathPoint.getHolonomicRotation());
        }

        /**
         * @param betterPathPoint path point to be flipped
         * @param alliance current alliance
         * @return a flipped path point to math current alliance
         */
        public static BetterPathPoint pathPointFlipper(BetterPathPoint betterPathPoint, Alliance alliance) {
            if (alliance == Alliance.Blue) {
                return betterPathPoint;
            } else if (alliance == Alliance.Red) {
                Translation2d transformedTranslation = new Translation2d(betterPathPoint.getPosition().getX(), fieldWidth - betterPathPoint.getPosition().getY());
                return new BetterPathPoint(transformedTranslation, betterPathPoint.getHeading().times(-1), betterPathPoint.getHolonomicRotation().times(-1));
            }
            return betterPathPoint;
        }
    }

    // Dimensions for community and charging station, including the tape.
    public static final class Community {
        // Region dimensions
        public static final double innerX = 0.0;
        public static final double midX =
                Units.inchesToMeters(132.375); // Tape to the left of charging station
        public static final double outerX =
                Units.inchesToMeters(193.25); // Tape to the right of charging station
        public static final double leftY = Units.feetToMeters(18.0);
        public static final double midY = leftY - Units.inchesToMeters(59.39) + tapeWidth;
        public static final double rightY = 0.0;
        public static final Translation2d[] regionCorners =
                new Translation2d[] {
                        new Translation2d(innerX, rightY),
                        new Translation2d(innerX, leftY),
                        new Translation2d(midX, leftY),
                        new Translation2d(midX, midY),
                        new Translation2d(outerX, midY),
                        new Translation2d(outerX, rightY),
                };

        // Charging station dimensions
        public static final double chargingStationLength = Units.inchesToMeters(76.125);
        public static final double chargingStationWidth = Units.inchesToMeters(97.25);
        public static final double chargingStationOuterX = outerX - tapeWidth;
        public static final double chargingStationInnerX =
                chargingStationOuterX - chargingStationLength;
        public static final double chargingStationLeftY = midY - tapeWidth;
        public static final double chargingStationRightY = chargingStationLeftY - chargingStationWidth;
        public static final Translation2d[] chargingStationCorners =
                new Translation2d[] {
                        new Translation2d(chargingStationInnerX, chargingStationRightY),
                        new Translation2d(chargingStationInnerX, chargingStationLeftY),
                        new Translation2d(chargingStationOuterX, chargingStationRightY),
                        new Translation2d(chargingStationOuterX, chargingStationLeftY)
                };

        // Cable bump
        public static final double cableBumpInnerX =
                innerX + Grids.outerX + Units.inchesToMeters(95.25);
        public static final double cableBumpOuterX = cableBumpInnerX + Units.inchesToMeters(7);
        public static final Translation2d[] cableBumpCorners =
                new Translation2d[] {
                        new Translation2d(cableBumpInnerX, 0.0),
                        new Translation2d(cableBumpInnerX, chargingStationRightY),
                        new Translation2d(cableBumpOuterX, 0.0),
                        new Translation2d(cableBumpOuterX, chargingStationRightY)
                };
    }

    // Dimensions for grids and nodes
    public static final class Grids {
        // X layout
        public static final double outerX = Units.inchesToMeters(54.25); // 1.38 m
        public static final double lowX =
                outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube nodes
        public static final double midX = outerX - Units.inchesToMeters(22.75);
        public static final double highX = outerX - Units.inchesToMeters(39.75);

        // Y layout
        public static final int nodeRowCount = 9;
        public static final double nodeFirstY = Units.inchesToMeters(20.19);
        public static final double nodeSeparationY = Units.inchesToMeters(22.0);

        public static final double[] nodeY = new double[] {
                        Units.inchesToMeters(20.19 + 22.0 * 0),
                        Units.inchesToMeters(20.19 + 22.0 * 1),
                        Units.inchesToMeters(20.19 + 22.0 * 2),
                        Units.inchesToMeters(20.19 + 22.0 * 3),
                        Units.inchesToMeters(20.19 + 22.0 * 4),
                        Units.inchesToMeters(20.19 + 22.0 * 5),
                        Units.inchesToMeters(20.19 + 22.0 * 6),
                        Units.inchesToMeters(20.19 + 22.0 * 7),
                        Units.inchesToMeters(20.19 + 22.0 * 8)
                };


        // Z layout
        public static final double cubeEdgeHigh = Units.inchesToMeters(3.0);
        public static final double cubeEdgeHighInches = 3.0;
        public static final double highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh;
        public static final double midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh;
        public static final double highConeZ = Units.inchesToMeters(46.0);
        public static final double midConeZ = Units.inchesToMeters(34.0);

        // Translations (all nodes in the same column/row have the same X/Y coordinate)
        public static final Translation2d[] lowTranslations = new Translation2d[nodeRowCount];
        public static final Translation2d[] midTranslations = new Translation2d[nodeRowCount];
        public static final Translation3d[] mid3dTranslations = new Translation3d[nodeRowCount];
        public static final Translation2d[] highTranslations = new Translation2d[nodeRowCount];
        public static final Translation3d[] high3dTranslations = new Translation3d[nodeRowCount];

        static {
            for (int i = 0; i < nodeRowCount; i++) {
                boolean isCube = i == 1 || i == 4 || i == 7;
                lowTranslations[i] = new Translation2d(lowX, nodeFirstY + nodeSeparationY * i);
                midTranslations[i] = new Translation2d(midX, nodeFirstY + nodeSeparationY * i);
                mid3dTranslations[i] =
                        new Translation3d(midX, nodeFirstY + nodeSeparationY * i, isCube ? midCubeZ : midConeZ);
                high3dTranslations[i] =
                        new Translation3d(
                                highX, nodeFirstY + nodeSeparationY * i, isCube ? highCubeZ : highConeZ);
                highTranslations[i] = new Translation2d(highX, nodeFirstY + nodeSeparationY * i);
            }
        }

        // Complex low layout (shifted to account for cube vs cone rows and wide edge nodes)
        public static final double complexLowXCones =
                outerX - Units.inchesToMeters(16.0) / 2.0; // Centered X under cone nodes
        public static final double complexLowXCubes = lowX; // Centered X under cube nodes
        public static final double complexLowOuterYOffset =
                nodeFirstY - Units.inchesToMeters(3.0) - (Units.inchesToMeters(25.75) / 2.0);

        public static final Translation2d[] complexLowTranslations =
                new Translation2d[] {
                        new Translation2d(complexLowXCones, nodeFirstY - complexLowOuterYOffset),
                        new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 1),
                        new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 2),
                        new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 3),
                        new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 4),
                        new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 5),
                        new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 6),
                        new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 7),
                        new Translation2d(
                                complexLowXCones, nodeFirstY + nodeSeparationY * 8 + complexLowOuterYOffset),
                };
    }

    // Dimensions for loading zone and substations, including the tape
    public static final class LoadingZone {
        // Region dimensions
        public static final double width = Units.inchesToMeters(99.0);
        public static final double innerX = FieldConstants.fieldLength;
        public static final double midX = fieldLength - Units.inchesToMeters(132.25);
        public static final double outerX = fieldLength - Units.inchesToMeters(264.25);
        public static final double leftY = FieldConstants.fieldWidth;
        public static final double midY = leftY - Units.inchesToMeters(50.5);
        public static final double rightY = leftY - width;
        public static final Translation2d[] regionCorners =
                new Translation2d[] {
                        new Translation2d(
                                midX, rightY), // Start at lower left next to border with opponent community
                        new Translation2d(midX, midY),
                        new Translation2d(outerX, midY),
                        new Translation2d(outerX, leftY),
                        new Translation2d(innerX, leftY),
                        new Translation2d(innerX, rightY),
                };

        // Double substation dimensions
        public static final double doubleSubstationLength = Units.inchesToMeters(14.0);
        public static final double doubleSubstationX = innerX - doubleSubstationLength;
        public static final double doubleSubstationShelfZ = Units.inchesToMeters(37.375);
        public static final double doubleSubstationCenterY = fieldWidth - Units.inchesToMeters(49.76);

        // Single substation dimensions
        public static final double singleSubstationWidth = Units.inchesToMeters(22.75);
        public static final double singleSubstationLeftX =
                FieldConstants.fieldLength - doubleSubstationLength - Units.inchesToMeters(88.77);
        public static final double singleSubstationCenterX =
                singleSubstationLeftX + (singleSubstationWidth / 2.0);
        public static final double singleSubstationRightX =
                singleSubstationLeftX + singleSubstationWidth;
        public static final Translation2d singleSubstationTranslation =
                new Translation2d(singleSubstationCenterX, leftY);

        public static final double singleSubstationHeight = Units.inchesToMeters(18.0);
        public static final double singleSubstationLowZ = Units.inchesToMeters(27.125);
        public static final double singleSubstationCenterZ =
                singleSubstationLowZ + (singleSubstationHeight / 2.0);
        public static final double singleSubstationHighZ =
                singleSubstationLowZ + singleSubstationHeight;
    }

    // Locations of staged game pieces
    public static final class StagingLocations {
        public static final double centerOffsetX = Units.inchesToMeters(47.36);
        public static final double positionX = fieldLength / 2.0 - Units.inchesToMeters(47.36);
        public static final double firstY = Units.inchesToMeters(36.19);
        public static final double separationY = Units.inchesToMeters(48.0);
        public static final Translation2d[] translations = new Translation2d[4];

        static {
            for (int i = 0; i < translations.length; i++) {
                translations[i] = new Translation2d(positionX, firstY + (i * separationY));
            }
        }
    }

    public static Pose2d allianceFlipper(Pose2d pose, Alliance alliance) {
        if (alliance == Alliance.Blue) {
            return pose;
        }
        Translation2d transformedTranslation =
                new Translation2d(pose.getTranslation().getX(), FieldConstants.fieldWidth - pose.getTranslation().getY());
        Rotation2d transformedHolonomicRotation = pose.getRotation().times(-1);
        return new Pose2d(transformedTranslation, transformedHolonomicRotation);
    }

    public static Pose3d allianceFlipper(Pose3d pose, Alliance alliance) {
        if (alliance == Alliance.Blue) {
            return pose;
        }
        Translation3d transformedTranslation =
                new Translation3d(pose.getTranslation().getX(), FieldConstants.fieldWidth - pose.getTranslation().getY(), pose.getTranslation().getZ());
        Rotation3d transformedHolonomicRotation = pose.getRotation().times(-1);
        return new Pose3d(transformedTranslation, transformedHolonomicRotation);
    }


    // AprilTag constants
    public static final double aprilTagWidth = Units.inchesToMeters(6.0);
    public static final AprilTagFieldLayout aprilTags =
            new AprilTagFieldLayout(
                    List.of(
                            new AprilTag(
                                    1,
                                    new Pose3d(
                                            Units.inchesToMeters(610.77),
                                            Grids.nodeY[1],
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d(0.0, 0.0, Math.PI))),
                            new AprilTag(
                                    2,
                                    new Pose3d(
                                            Units.inchesToMeters(610.77),
                                            Grids.nodeY[4],
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d(0.0, 0.0, Math.PI))),
                            new AprilTag(
                                    3,
                                    new Pose3d(
                                            Units.inchesToMeters(610.77),
                                            Grids.nodeY[7],
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d(0.0, 0.0, Math.PI))),
                            new AprilTag(
                                    4,
                                    new Pose3d(
                                            Units.inchesToMeters(636.96),
                                            LoadingZone.doubleSubstationCenterY,
                                            Units.inchesToMeters(27.38),
                                            new Rotation3d(0.0, 0.0, Math.PI))),
                            new AprilTag(
                                    5,
                                    new Pose3d(
                                            Units.inchesToMeters(14.25),
                                            LoadingZone.doubleSubstationCenterY,
                                            Units.inchesToMeters(27.38),
                                            new Rotation3d())),
                            new AprilTag(
                                    6,
                                    new Pose3d(
                                            Units.inchesToMeters(40.45),
                                            Grids.nodeY[7],
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d())),
                            new AprilTag(
                                    7,
                                    new Pose3d(
                                            Units.inchesToMeters(40.45),
                                            Grids.nodeY[4],
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d())),
                            new AprilTag(
                                    8,
                                    new Pose3d(
                                            Units.inchesToMeters(40.45),
                                            Grids.nodeY[1],
                                            Units.inchesToMeters(18.22),
                                            new Rotation3d()))),
                    fieldLength,
                    fieldWidth);
}