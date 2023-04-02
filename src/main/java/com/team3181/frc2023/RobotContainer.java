// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3181.frc2023;

import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.FieldConstants.AutoDrivePoints;
import com.team3181.frc2023.commands.*;
import com.team3181.frc2023.commands.autos.*;
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

  private SendableChooser<Command> autoChooser = new SendableChooser<>();
  private SendableChooser<Integer> positionChooser = new SendableChooser<>();
  private SendableChooser<Boolean> balanceChooser = new SendableChooser<>();
  private final HashMap<Command, Boolean> canBalanceMap = new HashMap<>();
  private final HashMap<Command, Boolean> needPositionMap = new HashMap<>();
  private final HashMap<Command, Boolean> canForwardBalance = new HashMap<>();

  public static boolean balanceForward = false;

  public RobotContainer() {
    autoConfig();
    competitionButtons();
//    testButtons();

    vision.setDataInterface(swerve::addVisionData);

    if (!RobotConstants.IS_TANK) swerve.setDefaultCommand(new SwerveDriveFieldXbox());
    if (RobotConstants.IS_TANK) Tank.getInstance().setDefaultCommand(new TankXbox());
  }

  private void testButtons() {
//      driverController.a().whileTrue(new SwerveAutoBalance(true));
    driverController.rightTrigger().whileTrue(new InstantCommand(swerve::zeroGyro));
    driverController.a().whileTrue(new InstantCommand(superstructure::collectGround)).whileFalse(new InstantCommand(superstructure::home));

    operatorController.a()
            .whileTrue(new InstantCommand(Superstructure.getInstance()::collectGround))
            .whileFalse(new InstantCommand(Superstructure.getInstance()::home));
    operatorController.rightTrigger()
            .whileTrue(new InstantCommand(Superstructure.getInstance()::exhaust))
            .whileFalse(new InstantCommand(Superstructure.getInstance()::home));
    operatorController.leftTrigger()
            .whileTrue(new InstantCommand(Superstructure.getInstance()::manual))
            .whileFalse(new InstantCommand(Superstructure.getInstance()::home));

    operatorController.povUp().whileTrue(objectiveTracker.shiftNodeCommand(Direction.UP));
    operatorController.povRight().whileTrue(objectiveTracker.shiftNodeCommand(Direction.RIGHT));
    operatorController.povDown().whileTrue(objectiveTracker.shiftNodeCommand(Direction.DOWN));
    operatorController.povLeft().whileTrue(objectiveTracker.shiftNodeCommand(Direction.LEFT));
//    driverController.a().whileTrue(new InstantCommand(objectiveTracker::toggleFilled));
  }

  private void competitionButtons() {
    /*
     DRIVER
     */
    driverController.x()
            .whileTrue(new InstantCommand(swerve::driveX));
//    driverController.a().whileTrue(new SwerveAutoBalance(true));
//    driverController.b().whileTrue(new SwerveAutoBalance(false));
    driverController.y()
            .whileTrue(new SwerveAutoScore())
            .whileFalse(new SuperstructureHome());
    driverController.b()
            .whileTrue(new SwerveAutoDoubleSubstationRight())
            .whileFalse(new SuperstructureHome());
    driverController.a()
            .whileTrue(new SwerveAutoDoubleSubstationLeft())
            .whileFalse(new SuperstructureHome());
    driverController.rightBumper()
            .whileTrue(new InstantCommand(swerve::zeroGyro));

    /*
     OPERATOR
     */
    operatorController.rightTrigger()
            .whileTrue(new InstantCommand(Superstructure.getInstance()::exhaust))
            .whileFalse(new InstantCommand(Superstructure.getInstance()::home));
    operatorController.leftTrigger()
            .whileTrue(new InstantCommand(Superstructure.getInstance()::manual))
            .whileFalse(new InstantCommand(Superstructure.getInstance()::home));
    operatorController.a()
            .whileTrue(new InstantCommand(Superstructure.getInstance()::collectGround))
            .whileFalse(new InstantCommand(Superstructure.getInstance()::home));
    operatorController.b()
            .whileTrue(new InstantCommand(Superstructure.getInstance()::collectMid))
            .whileFalse(new InstantCommand(Superstructure.getInstance()::home));
    operatorController.x()
            .whileTrue(new InstantCommand(Superstructure.getInstance()::objective))
            .whileFalse(new InstantCommand(Superstructure.getInstance()::home));
//    operatorController.y()
//            .whileTrue(new InstantCommand(Superstructure.getInstance()::setDemandLEDs));

    operatorController.povUp()
            .whileTrue(objectiveTracker.shiftNodeCommand(Direction.UP));
    operatorController.povRight()
            .whileTrue(objectiveTracker.shiftNodeCommand(Direction.RIGHT));
    operatorController.povDown()
            .whileTrue(objectiveTracker.shiftNodeCommand(Direction.DOWN));
    operatorController.povLeft()
            .whileTrue(objectiveTracker.shiftNodeCommand(Direction.LEFT));
    operatorController.leftBumper()
            .whileTrue(new InstantCommand(superstructure::zero));
//    operatorController.rightBumper().
//            whileTrue(new InstantCommand(objectiveTracker::toggleFilled));
//    operatorController.leftBumper().
//            whileTrue(new InstantCommand(objectiveTracker::toggleActive));
  }

  public void autoConfig() {
    balanceChooser.setDefaultOption("No Balance", false);
    balanceChooser.addOption("Yes Balance", true);

    positionChooser.setDefaultOption("Bottom Node", 0);
    positionChooser.addOption("Bottom Node + 1", 1);
    positionChooser.addOption("Bottom Node + 2", 2);
    positionChooser.addOption("Co-op Node", 3);
    positionChooser.addOption("Co-op Node + 1", 4);
    positionChooser.addOption("Co-op Node + 2", 5);
    positionChooser.addOption("Top Node", 6);
    positionChooser.addOption("Top Node + 1", 7);
    positionChooser.addOption("Top Node + 2", 8);

    Command wait = new WaitCommand(0);
    autoChooser.setDefaultOption("No auto", wait);
    canBalanceMap.put(wait, false);
    needPositionMap.put(wait, true);
    canForwardBalance.put(wait, true);

    Command balance = new AutoSwerveBalance();
    autoChooser.addOption("Place and Balance Climb", balance);
    canBalanceMap.put(balance, true);
    needPositionMap.put(balance, true);
    canForwardBalance.put(balance, false);

    Command threeTopBal = new AutoSwerveThreeBalance(true);
    autoChooser.addOption("3 Thing Top + Balance", threeTopBal);
    canBalanceMap.put(threeTopBal, true);
    needPositionMap.put(threeTopBal, false);
    canForwardBalance.put(threeTopBal, true);

    Command threeBottom = new AutoSwerveThree(false);
    autoChooser.addOption("3 Thing Bottom", threeBottom);
    canBalanceMap.put(threeBottom, false);
    needPositionMap.put(threeBottom, false);
    canForwardBalance.put(threeBottom, false);

    Command twoTop = new AutoSwerveTwo(true);
    autoChooser.addOption("2 Thing Top", twoTop);
    canBalanceMap.put(twoTop, true);
    needPositionMap.put(twoTop, false);
    canForwardBalance.put(twoTop, false);

    Command twoBottom = new AutoSwerveTwo(false);
    autoChooser.addOption("2 Thing Bottom", twoBottom);
    canBalanceMap.put(twoBottom, true);
    needPositionMap.put(twoBottom, false);
    canForwardBalance.put(twoBottom, false);

//    Command coneTop = new AutoSwervePath(Paths.TOP_CONE, new Objective(8, NodeLevel.HIGH));
//    autoChooser.addOption("1 Cone Top", coneTop);
//    canBalanceMap.put(coneTop, true);
//    needPositionMap.put(coneTop, false);
//    canForwardBalance.put(coneTop, false);

    Command coneTopPlusOne = new AutoSwervePath(Paths.TOP_CONE_PLUS_ONE, new Objective(8, NodeLevel.HIGH));
    autoChooser.addOption("1 Cone Top + 1", coneTopPlusOne);
    canBalanceMap.put(coneTopPlusOne, true);
    needPositionMap.put(coneTopPlusOne, false);
    canForwardBalance.put(coneTopPlusOne, false);

//    Command cubeTop = new AutoSwervePath(Paths.TOP_CUBE, new Objective(7, NodeLevel.HIGH));
//    autoChooser.addOption("1 Cube Top", cubeTop);
//    canBalanceMap.put(cubeTop, true);
//    needPositionMap.put(cubeTop, false);
//    canForwardBalance.put(cubeTop, false);

    Command cubeTopPlusOne = new AutoSwervePath(Paths.TOP_CUBE_PLUS_ONE, new Objective(7, NodeLevel.HIGH));
    autoChooser.addOption("1 Cube Top + 1", cubeTopPlusOne);
    canBalanceMap.put(cubeTopPlusOne, true);
    needPositionMap.put(cubeTopPlusOne, false);
    canForwardBalance.put(cubeTopPlusOne, false);

    Command coneBottomPlusOne = new AutoSwervePath(Paths.BOTTOM_CONE_PLUS_ONE, new Objective(0, NodeLevel.HIGH));
    autoChooser.addOption("1 Cone Bottom + 1", coneBottomPlusOne);
    canBalanceMap.put(coneBottomPlusOne, true);
    needPositionMap.put(coneBottomPlusOne, false);
    canForwardBalance.put(coneBottomPlusOne, false);

//    Command cubeBottom = new AutoSwervePath(Paths.BOTTOM_CUBE, new Objective(1, NodeLevel.HIGH));
//    autoChooser.addOption("1 Cube Bottom", cubeBottom);
//    canBalanceMap.put(cubeBottom, true);
//    needPositionMap.put(cubeBottom, false);
//    canForwardBalance.put(cubeBottom, false);

    Command cubeBottomPlusOne = new AutoSwervePath(Paths.BOTTOM_CUBE_PLUS_ONE, new Objective(1, NodeLevel.HIGH));
    autoChooser.addOption("1 Cube Bottom + 1", cubeBottomPlusOne);
    canBalanceMap.put(cubeBottomPlusOne, true);
    needPositionMap.put(cubeBottomPlusOne, false);
    canForwardBalance.put(cubeBottomPlusOne, false);

    SmartDashboard.putData("Auto Command", autoChooser);
    SmartDashboard.putData("Should Balance", balanceChooser);
    SmartDashboard.putData("Position", positionChooser);
  }

  // try to optimize memory usage
  public void killAuto() {
    autoChooser.close();
    positionChooser.close();
    balanceChooser.close();
    autoChooser = new SendableChooser<>();
    positionChooser = new SendableChooser<>();
    balanceChooser = new SendableChooser<>();
    SmartDashboard.putData("Auto Command", autoChooser);
    SmartDashboard.putData("Should Balance", balanceChooser);
    SmartDashboard.putData("Position", positionChooser);
  }

  public boolean canBalance() {
    return canBalanceMap.get(autoChooser.getSelected()) != null && canBalanceMap.get(autoChooser.getSelected());
  }

  public boolean needPosition() {
    return needPositionMap.get(autoChooser.getSelected()) != null && needPositionMap.get(autoChooser.getSelected());
  }

  public Command getAutonomousCommand() {
    endEffector.addGamePiece();
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
    balanceForward = canForwardBalance.get(autoChooser.getSelected());
    System.out.println(autoChooser.getSelected());
    System.out.println(positionChooser.getSelected());
    System.out.println(balanceChooser.getSelected());

    return autoChooser.getSelected();
  }
}