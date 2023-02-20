package com.team3181.lib.swerve;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class BetterPathPoint extends PathPoint {
    public BetterPathPoint(Translation2d position, Rotation2d heading, Rotation2d holonomicRotation, double velocityOverride) {
        super(position, heading, holonomicRotation, velocityOverride);
    }

    public BetterPathPoint(Translation2d position, Rotation2d heading, Rotation2d holonomicRotation) {
        super(position, heading, holonomicRotation);
    }

    public BetterPathPoint(Translation2d position, Rotation2d heading, double velocityOverride) {
        super(position, heading, velocityOverride);
    }

    public BetterPathPoint(Translation2d position, Rotation2d heading) {
        super(position, heading);
    }

    public Translation2d getPosition() {
        return position;
    }

    public Rotation2d getHeading() {
        return heading;
    }

    public Rotation2d getHolonomicRotation() {
        return holonomicRotation;
    }

    public double getVelocityOverride() {
        return velocityOverride;
    }
}