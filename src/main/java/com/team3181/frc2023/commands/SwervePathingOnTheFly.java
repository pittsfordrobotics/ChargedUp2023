package com.team3181.frc2023.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import com.team3181.frc2023.Constants.AutoConstants;
import com.team3181.frc2023.Constants.AutoConstants.AutoDrivePosition;
import com.team3181.frc2023.FieldConstants.AutoDrivePoints;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.lib.math.GeomUtil;
import com.team3181.lib.swerve.BetterPathPoint;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.ArrayList;


public class SwervePathingOnTheFly extends CommandBase {
    private final Swerve swerve = Swerve.getInstance();
    private PathPlannerTrajectory trajectory;
    private final BetterPathPoint[] pathPoint;
    private final AutoDrivePosition position;
    private final boolean simple;
    private final Timer timer = new Timer();
    private PathConstraints pathConstraints = AutoConstants.SLOW_SPEED;

    private final PIDController xController = new PIDController(AutoConstants.LINEAR_P, 0, 0);
    private final PIDController yController = new PIDController(AutoConstants.LINEAR_P, 0, 0);
    private final ProfiledPIDController rotController = new ProfiledPIDController(AutoConstants.ROT_P, 0, 0, AutoConstants.MAX_ROT_CONSTRAINTS);
    private final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(xController, yController, rotController);

    public SwervePathingOnTheFly(BetterPathPoint... pathPoint) {
        addRequirements(this.swerve);
        this.pathPoint = pathPoint;
        simple = false;
        position = null;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwervePathingOnTheFly(PathConstraints pathConstraints, BetterPathPoint... pathPoint) {
        addRequirements(this.swerve);
        this.pathPoint = pathPoint;
        this.pathConstraints = pathConstraints;
        simple = true;
        position = null;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwervePathingOnTheFly(AutoDrivePosition position, boolean simple) {
        addRequirements(this.swerve);
        this.pathPoint = null;
        this.simple = simple;
        this.position = position;
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public SwervePathingOnTheFly(boolean leftSubstation, boolean simple) {
        addRequirements(this.swerve);
        this.pathPoint = null;
        this.simple = simple;
        Alliance alliance = DriverStation.getAlliance();
        if (alliance == Alliance.Red) {
            if (leftSubstation) {
                this.position = AutoDrivePosition.DOUBLE_SUBSTATION_HIGH;
            }
            else {
                this.position = AutoDrivePosition.DOUBLE_SUBSTATION_LOW;
            }
        }
        else {
            if (leftSubstation) {
                this.position = AutoDrivePosition.DOUBLE_SUBSTATION_LOW;
            }
            else {
                this.position = AutoDrivePosition.DOUBLE_SUBSTATION_HIGH;
            }
        }
        rotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        ArrayList<PathPoint> adjustedPathPoints = new ArrayList<>();
        // adds path points and flips for correct alliance
        BetterPathPoint robotPoint = new BetterPathPoint(Swerve.getInstance().getPose().getTranslation(), Swerve.getInstance().getPose().getRotation(), Swerve.getInstance().getPose().getRotation());
        BetterPathPoint robotPointBlue = AutoDrivePoints.pathPointFlipper(new BetterPathPoint(Swerve.getInstance().getPose().getTranslation(), Swerve.getInstance().getPose().getRotation(), Swerve.getInstance().getPose().getRotation()), DriverStation.getAlliance());
        Objective objective = ObjectiveTracker.getInstance().getObjective();
        switch (position) {
            case NODE:
                BetterPathPoint node = AutoDrivePoints.nodeSelector(objective.nodeRow);
                if (objective.nodeLevel == NodeLevel.HYBRID) {
                    node = AutoDrivePoints.adjustNodeForHyrbid(node);
                }
                node = AutoDrivePoints.pathPointFlipper(node, DriverStation.getAlliance());
                if (simple || (robotPointBlue.getPosition().getX() > 1.8 && robotPointBlue.getPosition().getX() < 2.5)) {
                    BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, node);
                    BetterPathPoint updatedNode = new BetterPathPoint(node.getPosition(), headingCorrection.getHeading(), node.getHolonomicRotation());
                    adjustedPathPoints.add(headingCorrection);
                    adjustedPathPoints.add(updatedNode);
                }
                else {
                    if (robotPointBlue.getPosition().getY() > 7.2 && robotPointBlue.getPosition().getX() > 13.6) {
                        BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, AutoDrivePoints.pathPointFlipper(AutoDrivePoints.LOADING_STATION_TOP_EXIT, DriverStation.getAlliance()));
                        adjustedPathPoints.add(headingCorrection);
                        adjustedPathPoints.add(AutoDrivePoints.pathPointFlipper(AutoDrivePoints.LOADING_STATION_TOP_EXIT, DriverStation.getAlliance()));
                    }
                    else if (robotPointBlue.getPosition().getY() > 5.9 && robotPointBlue.getPosition().getX() > 13.6) {
                        BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, AutoDrivePoints.pathPointFlipper(AutoDrivePoints.LOADING_STATION_BOTTOM_EXIT, DriverStation.getAlliance()));
                        adjustedPathPoints.add(headingCorrection);
                        adjustedPathPoints.add(AutoDrivePoints.pathPointFlipper(AutoDrivePoints.LOADING_STATION_BOTTOM_EXIT, DriverStation.getAlliance()));
                    }
                    BetterPathPoint inner = objective.nodeRow > 3 ? AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_TOP_INNER, DriverStation.getAlliance()) : AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_BOTTOM_INNER, DriverStation.getAlliance());
                    if (robotPointBlue.getPosition().getX() > 5.26) {
                        double distanceTop = GeomUtil.distance(new Pose2d(AutoDrivePoints.COMMUNITY_TOP_EXIT.getPosition(), new Rotation2d()), new Pose2d(robotPointBlue.getPosition(), new Rotation2d()));
                        double distanceBottom = GeomUtil.distance(new Pose2d(AutoDrivePoints.COMMUNITY_BOTTOM_EXIT.getPosition(), new Rotation2d()), new Pose2d(robotPointBlue.getPosition(), new Rotation2d()));
                        BetterPathPoint entrance;
                        if (distanceTop > distanceBottom && Math.abs(distanceTop - distanceBottom) > 1) {
                            entrance = AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_BOTTOM_EXIT, DriverStation.getAlliance());
                            inner = AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_BOTTOM_INNER, DriverStation.getAlliance());
                        }
                        else if (distanceTop < distanceBottom && Math.abs(distanceBottom - distanceTop) > 1) {
                            entrance = AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_TOP_EXIT, DriverStation.getAlliance());
                            inner = AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_TOP_INNER, DriverStation.getAlliance());
                        }
                        else {
                            entrance = objective.nodeRow > 3 ? AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_TOP_EXIT, DriverStation.getAlliance()) : AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_BOTTOM_EXIT, DriverStation.getAlliance());
                            inner = objective.nodeRow > 3 ? AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_TOP_INNER, DriverStation.getAlliance()) : AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_BOTTOM_INNER, DriverStation.getAlliance());
                        }
                        if (adjustedPathPoints.size() == 0) {
                            BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, entrance);
                            adjustedPathPoints.add(headingCorrection);
                        }
                        adjustedPathPoints.add(entrance);
                    }

                    // slightly outside node
                    if (robotPointBlue.getPosition().getX() > 2) {
                        if (adjustedPathPoints.size() == 0 && robotPointBlue.getPosition().getY() < 5.5) {
                            double distanceTop = GeomUtil.distance(new Pose2d(AutoDrivePoints.COMMUNITY_TOP_INNER.getPosition(), new Rotation2d()), new Pose2d(robotPointBlue.getPosition(), new Rotation2d()));
                            double distanceBottom = GeomUtil.distance(new Pose2d(AutoDrivePoints.COMMUNITY_BOTTOM_INNER.getPosition(), new Rotation2d()), new Pose2d(robotPointBlue.getPosition(), new Rotation2d()));
                            if (distanceTop - distanceBottom > 1) {
                                inner = AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_BOTTOM_INNER, DriverStation.getAlliance());
                            }
                            else if (distanceBottom - distanceTop > 1) {
                                inner = AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_TOP_INNER, DriverStation.getAlliance());
                            }
                            else {
                                inner = objective.nodeRow > 3 ? AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_TOP_INNER, DriverStation.getAlliance()) : AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_BOTTOM_INNER, DriverStation.getAlliance());
                            }
                            BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, inner);
                            adjustedPathPoints.add(headingCorrection);
                        }
                        adjustedPathPoints.add(inner);
                    }
                    // inside node
                    if (robotPointBlue.getPosition().getX() > 1) {
                        BetterPathPoint updatedNode;
                        BetterPathPoint nodeBlue = AutoDrivePoints.pathPointFlipper(node, DriverStation.getAlliance());
                        if ((robotPointBlue.getPosition().getY() > 2.78 && nodeBlue.getPosition().getY() < 2.78) || (robotPointBlue.getPosition().getY() < 2.78 && nodeBlue.getPosition().getY() > 2.78)) {
                            System.out.println("Going mid");
                            BetterPathPoint mid = AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_MID_INNER, DriverStation.getAlliance());
                            if (adjustedPathPoints.size() == 0 && robotPointBlue.getPosition().getY() < 5.5) {
                                BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, mid);
                                adjustedPathPoints.add(headingCorrection);
                                adjustedPathPoints.add(new BetterPathPoint(mid.getPosition(), AutoDrivePoints.updateHeading(mid, node).getHeading(), mid.getHolonomicRotation()));
                                updatedNode = new BetterPathPoint(node.getPosition(), AutoDrivePoints.updateHeading(mid, node).getHeading(), node.getHolonomicRotation());
                            } else {
                                adjustedPathPoints.add(new BetterPathPoint(mid.getPosition(), AutoDrivePoints.updateHeading(mid, node).getHeading(), mid.getHolonomicRotation()));
                                updatedNode = new BetterPathPoint(node.getPosition(), AutoDrivePoints.updateHeading(mid, node).getHeading(), node.getHolonomicRotation());
                            }
                            adjustedPathPoints.add(updatedNode);
                        }
                        else {
                            if (adjustedPathPoints.size() == 0 && robotPointBlue.getPosition().getY() < 5.5) {
                                BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, node);
                                adjustedPathPoints.add(headingCorrection);
                                updatedNode = new BetterPathPoint(node.getPosition(), headingCorrection.getHeading(), node.getHolonomicRotation());
                            } else {
                                updatedNode = new BetterPathPoint(node.getPosition(), AutoDrivePoints.updateHeading(inner, node).getHeading(), node.getHolonomicRotation());
                            }
                        }
                        adjustedPathPoints.add(updatedNode);
                    }
                }
                break;
            case SINGLE_SUBSTATION:
//                TODO: this won't be implemented for right now
                break;
            case DOUBLE_SUBSTATION_HIGH:
                if (simple) {
                    BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.LOADING_STATION_TOP_INNER), DriverStation.getAlliance()));
                    adjustedPathPoints.add(headingCorrection);
                    adjustedPathPoints.add(AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.LOADING_STATION_TOP_INNER), DriverStation.getAlliance()));
                }
                else {
                    BetterPathPoint entrance = AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_BOTTOM_EXIT), DriverStation.getAlliance());
                    if (robotPointBlue.getPosition().getX() < 2 && robotPointBlue.getPosition().getY() < 5.5) {
                        double distanceTop = GeomUtil.distance(new Pose2d(AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_TOP_INNER).getPosition(), new Rotation2d()), new Pose2d(robotPointBlue.getPosition(), new Rotation2d()));
                        double distanceBottom = GeomUtil.distance(new Pose2d(AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_BOTTOM_INNER).getPosition(), new Rotation2d()), new Pose2d(robotPointBlue.getPosition(), new Rotation2d()));
                        BetterPathPoint inner;
                        if (distanceTop > distanceBottom) {
                            inner = AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_BOTTOM_INNER), DriverStation.getAlliance());
                            entrance = AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_BOTTOM_EXIT), DriverStation.getAlliance());
                        } else {
                            inner = AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_TOP_INNER), DriverStation.getAlliance());
                            entrance = AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_TOP_EXIT), DriverStation.getAlliance());
                        }
                        BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, inner);
                        adjustedPathPoints.add(headingCorrection);
                    }
                    if (robotPointBlue.getPosition().getX() < 5.26) {
                        if (adjustedPathPoints.size() == 0) {
                            double distanceTop = GeomUtil.distance(new Pose2d(AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_TOP_EXIT).getPosition(), new Rotation2d()), new Pose2d(robotPointBlue.getPosition(), new Rotation2d()));
                            double distanceBottom = GeomUtil.distance(new Pose2d(AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_BOTTOM_EXIT).getPosition(), new Rotation2d()), new Pose2d(robotPointBlue.getPosition(), new Rotation2d()));
                            if (Math.abs(distanceTop) > Math.abs(distanceBottom)) {
                                entrance = AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_BOTTOM_EXIT), DriverStation.getAlliance());
                            } else {
                                entrance = AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_TOP_EXIT), DriverStation.getAlliance());
                            }
                            BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, entrance);
                            adjustedPathPoints.add(headingCorrection);
                        }
                        adjustedPathPoints.add(entrance);
                    }
                    if (robotPointBlue.getPosition().getY() > 7.2 && robotPointBlue.getPosition().getX() > 10.25) {
                        if (adjustedPathPoints.size() == 0) {
                            BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.LOADING_STATION_TOP_INNER), DriverStation.getAlliance()));
                            adjustedPathPoints.add(headingCorrection);
                        }
                        adjustedPathPoints.add(AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.LOADING_STATION_TOP_INNER), DriverStation.getAlliance()));
                    }
                    else if (robotPointBlue.getPosition().getY() > 5.9 && robotPointBlue.getPosition().getX() > 10.25) {
                        if (adjustedPathPoints.size() == 0) {
                            BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.LOADING_STATION_BOTTOM_INNER), DriverStation.getAlliance()));
                            adjustedPathPoints.add(headingCorrection);
                        }
                        adjustedPathPoints.add(AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.LOADING_STATION_BOTTOM_INNER), DriverStation.getAlliance()));
                    }
                }
                break;
            case DOUBLE_SUBSTATION_LOW:
                if (simple) {
                    BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.LOADING_STATION_BOTTOM_INNER), DriverStation.getAlliance()));
                    adjustedPathPoints.add(headingCorrection);
                    adjustedPathPoints.add(AutoDrivePoints.pathPointFlipper(AutoDrivePoints.leavingCommunity(AutoDrivePoints.LOADING_STATION_BOTTOM_INNER), DriverStation.getAlliance()));
                }
                break;
            default:
                for (int i = 1; i < pathPoint.length + 1; i++) {
                    if (i == 1) {
                        BetterPathPoint headingCorrection = AutoDrivePoints.updateHeading(robotPoint, AutoDrivePoints.pathPointFlipper(pathPoint[i - 1], DriverStation.getAlliance()));
                        adjustedPathPoints.add(headingCorrection);
                    }
                    adjustedPathPoints.add(AutoDrivePoints.pathPointFlipper(pathPoint[i - 1], DriverStation.getAlliance()));
                }
                break;
        }

        // creates trajectory
        try {
            this.trajectory = PathPlanner.generatePath(
                    pathConstraints,
                    adjustedPathPoints
            );
            xController.reset();
            yController.reset();
            rotController.reset(trajectory.getInitialState().holonomicRotation.getRadians());
        } catch (Exception ignored) {}
        // path following setup

        timer.restart();
    }

    @Override
    public void execute() {
        try {
            PathPlannerState state = (PathPlannerState) trajectory.sample(timer.get());
            ChassisSpeeds speeds = holonomicDriveController.calculate(swerve.getPose(), state, state.holonomicRotation);
            swerve.setChassisSpeeds(speeds, false);
        } catch (Exception ignored) {}
    }

    @Override
    public boolean isFinished() {
        return trajectory == null || timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        swerve.stopMotors();
    }
}