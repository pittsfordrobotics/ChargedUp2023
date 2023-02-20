// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3181.frc2023;

import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.FieldConstants.AutoDrivePoints;
import com.team3181.frc2023.commands.*;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Direction;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.frc2023.subsystems.tank.Tank;
import com.team3181.lib.controller.BetterXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {
  private ObjectiveTracker objectiveTracker = ObjectiveTracker.getInstance();

  private final BetterXboxController driverController = new BetterXboxController(0, BetterXboxController.Humans.DRIVER);
  private final BetterXboxController operatorController = new BetterXboxController(1, BetterXboxController.Humans.OPERATOR);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final SendableChooser<Pose2d> positionChooser = new SendableChooser<>();

  public RobotContainer() {
    autoConfig();
    driverDashboardSetup();

//      competitionButtons();
      testButtons();

    if (!RobotConstants.IS_TANK) Swerve.getInstance().setDefaultCommand(new SwerveDriveFieldXbox());
    if (RobotConstants.IS_TANK) Tank.getInstance().setDefaultCommand(new TankXbox());
  }

  private void driverDashboardSetup() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  }

  private void testButtons() {
//    driverController.a().whileTrue(new InstantCommand(Swerve.getInstance()::zeroGyro));
//    driverController.x().whileTrue(new InstantCommand(Swerve.getInstance()::driveX));
    driverController.a().whileTrue(new SwervePathingOnTheFly(
            AutoDrivePoints.LOADING_STATION_TOP_EXIT,
            AutoDrivePoints.COMMUNITY_TOP_EXIT,
            AutoDrivePoints.COMMUNITY_TOP_INNER,
            AutoDrivePoints.nodeSelector(5))
    );
    driverController.b().whileTrue(new SwervePathingOnTheFly(
            AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_BOTTOM_INNER),
            AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_BOTTOM_EXIT),
            AutoDrivePoints.leavingCommunity(AutoDrivePoints.LOADING_STATION_TOP_EXIT),
            AutoDrivePoints.leavingCommunity(AutoDrivePoints.LOADING_STATION_TOP_INNER))
    );
    operatorController.povUp().whileTrue(objectiveTracker.shiftNodeCommand(Direction.UP));
    operatorController.povRight().whileTrue(objectiveTracker.shiftNodeCommand(Direction.RIGHT));
    operatorController.povDown().whileTrue(objectiveTracker.shiftNodeCommand(Direction.DOWN));
    operatorController.povLeft().whileTrue(objectiveTracker.shiftNodeCommand(Direction.LEFT));
  }

  private void competitionButtons() {}

  private void demoButtons() {}

  private void autoConfig() {
    autoChooser.setDefaultOption("No auto", new WaitCommand(0));
//    autoChooser.addOption("Test", new SwervePathing(Paths., true));
    autoChooser.addOption("Drop Climb", new DropClimb(new Pose2d(new Translation2d(1.78,2.55), Rotation2d.fromDegrees(180))));
    autoChooser.addOption("3 thing", new AutoCollectAndGo());

    SmartDashboard.putData("Auto Command", autoChooser);
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}