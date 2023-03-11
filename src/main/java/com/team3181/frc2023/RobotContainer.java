// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3181.frc2023;

import com.team3181.frc2023.Constants.AutoConstants.AutoDrivePosition;
import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.FieldConstants.AutoDrivePoints;
import com.team3181.frc2023.commands.*;
import com.team3181.frc2023.subsystems.Superstructure;
import com.team3181.frc2023.subsystems.endeffector.EndEffector;
import com.team3181.frc2023.subsystems.fourbar.FourBar;
import com.team3181.frc2023.subsystems.leds.LEDs;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Direction;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.frc2023.subsystems.tank.Tank;
import com.team3181.frc2023.subsystems.vision.Vision;
import com.team3181.lib.controller.BetterXboxController;
import com.team3181.lib.swerve.BetterPathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.HashMap;

public class RobotContainer {
  private final Swerve swerve = Swerve.getInstance();
  private final FourBar fourBar = FourBar.getInstance();
  private final EndEffector endEffector = EndEffector.getInstance();
  private final Vision vision = Vision.getInstance();
  private final ObjectiveTracker objectiveTracker = ObjectiveTracker.getInstance();
  private final Superstructure superstructure = Superstructure.getInstance();
  private final LEDs leds = LEDs.getInstance();

  private final BetterXboxController driverController = new BetterXboxController(0, BetterXboxController.Humans.DRIVER);
  private final BetterXboxController operatorController = new BetterXboxController(1, BetterXboxController.Humans.OPERATOR);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final SendableChooser<BetterPathPoint> positionChooser = new SendableChooser<>();
  private final SendableChooser<Boolean> balanceChooser = new SendableChooser<>();
  private final HashMap<Command, Boolean> canBalanceMap = new HashMap<>();
  private final HashMap<Command, Boolean> needPositionMap = new HashMap<>();

  public RobotContainer() {
    autoConfig();

      competitionButtons();
//      testButtons();

    if (!RobotConstants.IS_TANK) swerve.setDefaultCommand(new SwerveDriveFieldXbox());
    if (RobotConstants.IS_TANK) Tank.getInstance().setDefaultCommand(new TankXbox());
  }

  private void testButtons() {
    driverController.a().whileTrue(new SwervePathingOnTheFly(AutoDrivePosition.NODE, false));
//    driverController.a().onTrue(new InstantCommand(endEffector::intake)).onFalse(new InstantCommand(endEffector::idle));
//    driverController.x().onTrue(new InstantCommand(endEffector::exhaust)).onFalse(new InstantCommand(endEffector::idle));
//    driverController.rightTrigger().whileTrue(new InstantCommand(swerve::zeroGyro));
//    driverController.a().whileTrue(new SwervePathing(Paths.TEST_ON_THE_FLY, false));
//    driverController.a().whileTrue(new SwervePathingOnTheFly(
//            AutoDrivePoints.LOADING_STATION_TOP_INNER,
//            AutoDrivePoints.LOADING_STATION_TOP_EXIT,
//            AutoDrivePoints.COMMUNITY_TOP_EXIT,
//            AutoDrivePoints.COMMUNITY_TOP_INNER,
//            AutoDrivePoints.nodeSelector(5))
//    );
//    driverController.b().whileTrue(new SwervePathingOnTheFly(
//            AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_BOTTOM_INNER),
//            AutoDrivePoints.leavingCommunity(AutoDrivePoints.COMMUNITY_BOTTOM_EXIT),
//            AutoDrivePoints.leavingCommunity(AutoDrivePoints.LOADING_STATION_TOP_EXIT),
//            AutoDrivePoints.leavingCommunity(AutoDrivePoints.LOADING_STATION_TOP_INNER))
//    );
//driverController.getRightTriggerAxis()
//    driverController.leftBumper()
//            .whileTrue(new RepeatCommand(new InstantCommand(() -> fourBar.setRotations(FourBar.getInstance().solve(new Translation2d(ArmPositions.SWEEP_MAX.getX(), ArmPositions.SWEEP_MIN.getY()), false, true)))))
//            .whileFalse(new InstantCommand(fourBar::hold));
//    driverController.rightTrigger()
//            .whileTrue(new RepeatCommand(new InstantCommand(() -> fourBar.setRotations(new Rotation2d[] {ArmPositions.STORAGE_SHOULDER, ArmPositions.STORAGE_ELBOW}))))
//            .whileFalse(new InstantCommand(fourBar::hold));
//    driverController.leftBumper().whileTrue(new RepeatCommand(new InstantCommand(() -> fourBar.setRotations(new Rotation2d[] {ArmPositions.GROUND_PICKUP_SHOULDER, ArmPositions.GROUND_PICKUP_ELBOW}))));
//    driverController.b().whileTrue(new InstantCommand(() -> fourBar.setRotations(new Rotation2d[] {ArmPositions.STORAGE_SHOULDER, ArmPositions.STORAGE_ELBOW})));

    driverController.a()
            .whileTrue(new RepeatCommand(new InstantCommand(() -> fourBar.setArmVoltage(0, 2))))
            .whileFalse(new InstantCommand(() -> fourBar.setArmVoltage(0, 0)));
    driverController.b()
            .whileTrue(new RepeatCommand(new InstantCommand(() -> fourBar.setArmVoltage(0, -2))))
            .whileFalse(new InstantCommand(() -> fourBar.setArmVoltage(0, 0)));
    driverController.x()
            .whileTrue(new RepeatCommand(new InstantCommand(() -> fourBar.setArmVoltage(1, 2))))
            .whileFalse(new InstantCommand(() -> fourBar.setArmVoltage(1, 0)));
    driverController.y()
            .whileTrue(new RepeatCommand(new InstantCommand(() -> fourBar.setArmVoltage(1, -2))))
            .whileFalse(new InstantCommand(() -> fourBar.setArmVoltage(1, 0)));
    driverController.rightBumper()
            .whileTrue(new RepeatCommand(new InstantCommand(endEffector::intake)))
            .whileFalse(new InstantCommand(endEffector::idle));
    driverController.leftBumper()
            .whileTrue(new RepeatCommand(new InstantCommand(endEffector::exhaust)))
            .whileFalse(new InstantCommand(endEffector::idle));
//    driverController.rightBumper().whileTrue(new InstantCommand(endEffector::exhaust)).whileFalse(new InstantCommand(endEffector::idle));

    operatorController.povUp().whileTrue(objectiveTracker.shiftNodeCommand(Direction.UP));
    operatorController.povRight().whileTrue(objectiveTracker.shiftNodeCommand(Direction.RIGHT));
    operatorController.povDown().whileTrue(objectiveTracker.shiftNodeCommand(Direction.DOWN));
    operatorController.povLeft().whileTrue(objectiveTracker.shiftNodeCommand(Direction.LEFT));
    operatorController.rightBumper().whileTrue(new InstantCommand(objectiveTracker::toggleFilled));
    operatorController.leftBumper().whileTrue(new InstantCommand(objectiveTracker::toggleActive));
  }

  private void competitionButtons() {
    /*
     DRIVER
     */
    driverController.x()
            .whileTrue(new InstantCommand(swerve::driveX));
    driverController.rightBumper()
            .whileTrue(new InstantCommand(swerve::zeroGyro));

    /*
     OPERATOR
     */
    operatorController.rightBumper()
            .whileTrue(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::exhaust)))
            .whileFalse(new InstantCommand(Superstructure.getInstance()::idle));
    operatorController.leftBumper()
            .whileTrue(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::manual)))
            .whileFalse(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::idle)));
    operatorController.a()
            .whileTrue(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::collectGround)))
            .whileFalse(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::home)));
    operatorController.b()
            .whileTrue(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::collectMid)))
            .whileFalse(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::home)));
    operatorController.x()
            .whileTrue(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::objective)))
            .whileFalse(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::home)));


    operatorController.povUp()
            .whileTrue(objectiveTracker.shiftNodeCommand(Direction.UP));
    operatorController.povRight()
            .whileTrue(objectiveTracker.shiftNodeCommand(Direction.RIGHT));
    operatorController.povDown()
            .whileTrue(objectiveTracker.shiftNodeCommand(Direction.DOWN));
    operatorController.povLeft()
            .whileTrue(objectiveTracker.shiftNodeCommand(Direction.LEFT));
    operatorController.rightBumper().
            whileTrue(new InstantCommand(objectiveTracker::toggleFilled));
    operatorController.leftBumper().
            whileTrue(new InstantCommand(objectiveTracker::toggleActive));
  }

  private void demoButtons() {}

  private void autoConfig() {
    balanceChooser.setDefaultOption("No Balance", false);
    balanceChooser.addOption("Yes Balance", true);

    positionChooser.setDefaultOption("Bottom Node", AutoDrivePoints.nodeSelector(0));

    autoChooser.setDefaultOption("No auto", new WaitCommand(0));
    canBalanceMap.put(new WaitCommand(0), false);
    needPositionMap.put(new WaitCommand(0), true);

    autoChooser.addOption("Place and Balance Climb", new AutoSwerveBalance());
    canBalanceMap.put(new AutoSwerveBalance(), true);
    needPositionMap.put(new AutoSwerveBalance(), true);

    autoChooser.addOption("3 Thing High", new AutoSwerveHighThree());
    canBalanceMap.put(new AutoSwerveHighThree(), true);
    needPositionMap.put(new AutoSwerveHighThree(), false);

    SmartDashboard.putData("Auto Command", autoChooser);
    SmartDashboard.putData("Should Balance", balanceChooser);
    SmartDashboard.putData("Position", positionChooser);
  }

  private void configureButtonBindings() {}

  public boolean canBalance() {
    return canBalanceMap.get(autoChooser.getSelected()) != null && canBalanceMap.get(autoChooser.getSelected());
  }

  public boolean needPosition() {
    return needPositionMap.get(autoChooser.getSelected()) != null && needPositionMap.get(autoChooser.getSelected());
  }

  public Command getAutonomousCommand() {
    if (needPosition()) {
      swerve.resetPose(new Pose2d(AutoDrivePoints.pathPointFlipper(positionChooser.getSelected(), DriverStation.getAlliance()).getPosition(), Rotation2d.fromDegrees(-180)));
    }
    if (canBalance() && balanceChooser.getSelected()) {
      Paths.EVENT_MAP = Paths.EVENT_MAP_BALANCE;
    }
    else {
      Paths.EVENT_MAP = Paths.EVENT_MAP_NO_BALANCE;
    }
    return autoChooser.getSelected();
  }
}