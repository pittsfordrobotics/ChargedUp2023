// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3181.frc2023;

import com.team3181.frc2023.Constants.AutoConstants.AutoDrivePosition;
import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.FieldConstants.AutoDrivePoints;
import com.team3181.frc2023.commands.SwerveDriveFieldXbox;
import com.team3181.frc2023.commands.SwervePathingOnTheFly;
import com.team3181.frc2023.commands.TankXbox;
import com.team3181.frc2023.commands.autos.AutoSwerveBalance;
import com.team3181.frc2023.commands.autos.AutoSwervePath;
import com.team3181.frc2023.commands.autos.AutoSwerveThree;
import com.team3181.frc2023.commands.autos.AutoSwerveTwo;
import com.team3181.frc2023.subsystems.Superstructure;
import com.team3181.frc2023.subsystems.endeffector.EndEffector;
import com.team3181.frc2023.subsystems.fourbar.FourBar;
import com.team3181.frc2023.subsystems.leds.LEDs;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Direction;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.NodeLevel;
import com.team3181.frc2023.subsystems.objectivetracker.ObjectiveTracker.Objective;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.frc2023.subsystems.tank.Tank;
import com.team3181.frc2023.subsystems.vision.Vision;
import com.team3181.lib.controller.BetterXboxController;
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
  private final SendableChooser<Integer> positionChooser = new SendableChooser<>();
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

    positionChooser.setDefaultOption("Bottom Node", 0);
    positionChooser.setDefaultOption("Bottom Node + 1", 1);
    positionChooser.setDefaultOption("Bottom Node + 2", 2);
    positionChooser.setDefaultOption("Co-op Node", 3);
    positionChooser.setDefaultOption("Co-op Node + 1", 4);
    positionChooser.setDefaultOption("Co-op Node + 2", 5);
    positionChooser.setDefaultOption("Top Node", 6);
    positionChooser.setDefaultOption("Top Node + 1", 7);
    positionChooser.setDefaultOption("Top Node + 2", 8);

    autoChooser.setDefaultOption("No auto", new WaitCommand(0));
    canBalanceMap.put(new WaitCommand(0), false);
    needPositionMap.put(new WaitCommand(0), true);

    autoChooser.addOption("Place and Balance Climb", new AutoSwerveBalance());
    canBalanceMap.put(new AutoSwerveBalance(), true);
    needPositionMap.put(new AutoSwerveBalance(), true);

    autoChooser.addOption("3 Thing Top", new AutoSwerveThree(true));
    canBalanceMap.put(new AutoSwerveThree(true), true);
    needPositionMap.put(new AutoSwerveThree(true), false);

    autoChooser.addOption("3 Thing Bottom", new AutoSwerveThree(false));
    canBalanceMap.put(new AutoSwerveThree(false), true);
    needPositionMap.put(new AutoSwerveThree(false), false);

    autoChooser.addOption("2 Thing Top", new AutoSwerveTwo(true));
    canBalanceMap.put(new AutoSwerveTwo(true), false);
    needPositionMap.put(new AutoSwerveTwo(true), false);

    autoChooser.addOption("2 Thing Bottom", new AutoSwerveTwo(false));
    canBalanceMap.put(new AutoSwerveTwo(false), false);
    needPositionMap.put(new AutoSwerveTwo(false), false);

    autoChooser.addOption("1 Cone Top", new AutoSwervePath(Paths.TOP_CONE, new Objective(8, NodeLevel.HIGH)));
    canBalanceMap.put(new AutoSwervePath(Paths.TOP_CONE, new Objective(8, NodeLevel.HIGH)), true);
    needPositionMap.put(new AutoSwervePath(Paths.TOP_CONE, new Objective(8, NodeLevel.HIGH)), false);

    autoChooser.addOption("1 Cone Top + 1", new AutoSwervePath(Paths.TOP_CONE_PLUS_ONE, new Objective(8, NodeLevel.HIGH)));
    canBalanceMap.put(new AutoSwervePath(Paths.TOP_CONE_PLUS_ONE, new Objective(8, NodeLevel.HIGH)), true);
    needPositionMap.put(new AutoSwervePath(Paths.TOP_CONE_PLUS_ONE, new Objective(8, NodeLevel.HIGH)), false);

    autoChooser.addOption("1 Cube Top", new AutoSwervePath(Paths.TOP_CUBE, new Objective(7, NodeLevel.HIGH)));
    canBalanceMap.put(new AutoSwervePath(Paths.TOP_CUBE, new Objective(7, NodeLevel.HIGH)), true);
    needPositionMap.put(new AutoSwervePath(Paths.TOP_CUBE, new Objective(7, NodeLevel.HIGH)), false);

    autoChooser.addOption("1 Cube Top + 1", new AutoSwervePath(Paths.TOP_CUBE_PLUS_ONE, new Objective(7, NodeLevel.HIGH)));
    canBalanceMap.put(new AutoSwervePath(Paths.TOP_CUBE_PLUS_ONE, new Objective(7, NodeLevel.HIGH)), true);
    needPositionMap.put(new AutoSwervePath(Paths.TOP_CUBE_PLUS_ONE, new Objective(7, NodeLevel.HIGH)), false);

    autoChooser.addOption("1 Cone Bottom", new AutoSwervePath(Paths.BOTTOM_CONE, new Objective(0, NodeLevel.HIGH)));
    canBalanceMap.put(new AutoSwervePath(Paths.BOTTOM_CONE, new Objective(0, NodeLevel.HIGH)), true);
    needPositionMap.put(new AutoSwervePath(Paths.BOTTOM_CONE, new Objective(0, NodeLevel.HIGH)), false);

    autoChooser.addOption("1 Cone Bottom + 1", new AutoSwervePath(Paths.BOTTOM_CONE_PLUS_ONE, new Objective(1, NodeLevel.HIGH)));
    canBalanceMap.put(new AutoSwervePath(Paths.BOTTOM_CONE_PLUS_ONE, new Objective(1, NodeLevel.HIGH)), true);
    needPositionMap.put(new AutoSwervePath(Paths.BOTTOM_CONE_PLUS_ONE, new Objective(1, NodeLevel.HIGH)), false);

    autoChooser.addOption("1 Cube Bottom", new AutoSwervePath(Paths.BOTTOM_CUBE, new Objective(1, NodeLevel.HIGH)));
    canBalanceMap.put(new AutoSwervePath(Paths.BOTTOM_CUBE, new Objective(1, NodeLevel.HIGH)), true);
    needPositionMap.put(new AutoSwervePath(Paths.BOTTOM_CUBE, new Objective(1, NodeLevel.HIGH)), false);

    autoChooser.addOption("1 Cube Bottom + 1", new AutoSwervePath(Paths.BOTTOM_CUBE_PLUS_ONE, new Objective(1, NodeLevel.HIGH)));
    canBalanceMap.put(new AutoSwervePath(Paths.BOTTOM_CUBE_PLUS_ONE, new Objective(1, NodeLevel.HIGH)), true);
    needPositionMap.put(new AutoSwervePath(Paths.BOTTOM_CUBE_PLUS_ONE, new Objective(1, NodeLevel.HIGH)), false);

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
    objectiveTracker.setAutomated();
    objectiveTracker.updateObjective(new Objective(positionChooser.getSelected(), NodeLevel.HIGH));
    if (needPosition()) {
      swerve.resetPose(new Pose2d(AutoDrivePoints.pathPointFlipper(AutoDrivePoints.nodeSelector(positionChooser.getSelected()), DriverStation.getAlliance()).getPosition(), Rotation2d.fromDegrees(-180)));
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