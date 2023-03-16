// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3181.frc2023;

import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.FieldConstants.AutoDrivePoints;
import com.team3181.frc2023.commands.SwerveDriveFieldXbox;
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

//      competitionButtons();
      testButtons();

    if (!RobotConstants.IS_TANK) swerve.setDefaultCommand(new SwerveDriveFieldXbox());
    if (RobotConstants.IS_TANK) Tank.getInstance().setDefaultCommand(new TankXbox());
  }

  private void testButtons() {
//    driverController.a().whileTrue(new SwervePathingOnTheFly(AutoDrivePosition.NODE, false));
//    driverController.a().whileTrue(new SwerveAutoScore());
//    driverController.a()
//            .whileTrue(new SwerveAutoDoubleSubstationLow());
    //    driverController.y()
//            .whileTrue(new SwerveAutoScore())
//            .whileFalse(new SuperstructureHome());
//    driverController.b()
//            .whileTrue(new SwerveAutoDoubleSubstationHigh())
//            .whileFalse(new SuperstructureHome());
//    driverController.a()
//            .whileTrue(new SwerveAutoDoubleSubstationLow())
//            .whileFalse(new SuperstructureHome());
//    driverController.a().whileTrue(new SwerveAutoBalance(true));
//    driverController.a().onTrue(new InstantCommand(endEffector::intake)).onFalse(new InstantCommand(endEffector::idle));
//    driverController.x().onTrue(new InstantCommand(endEffector::exhaust)).onFalse(new InstantCommand(endEffector::idle));
    driverController.rightTrigger().whileTrue(new InstantCommand(swerve::zeroGyro));
//
//    driverController.a().whileTrue(new InstantCommand(superstructure::collectGround))
//            .whileFalse(new InstantCommand(Superstructure.getInstance()::home));
//    driverController.b().whileTrue(new InstantCommand(superstructure::collectMid))
//            .whileFalse(new InstantCommand(Superstructure.getInstance()::home));
//    driverController.x().whileTrue(new InstantCommand(superstructure::objective))
//            .whileFalse(new InstantCommand(Superstructure.getInstance()::home));
//    driverController.b().whileTrue(new InstantCommand(() -> leds.setLEDMode(LEDModes.RAINBOW)));
//    driverController.a()
//            .whileTrue(new RepeatCommand(new InstantCommand(() -> fourBar.setArmVoltage(0, 2))))
//            .whileFalse(new InstantCommand(() -> fourBar.setArmVoltage(0, 0)));
//    driverController.b()
//            .whileTrue(new RepeatCommand(new InstantCommand(() -> fourBar.setArmVoltage(0, -2))))
//            .whileFalse(new InstantCommand(() -> fourBar.setArmVoltage(0, 0)));
//    driverController.x()
//            .whileTrue(new RepeatCommand(new InstantCommand(() -> fourBar.setArmVoltage(1, 2))))
//            .whileFalse(new InstantCommand(() -> fourBar.setArmVoltage(1, 0)));
//    driverController.y()
//            .whileTrue(new RepeatCommand(new InstantCommand(() -> fourBar.setArmVoltage(1, -2))))
//            .whileFalse(new InstantCommand(() -> fourBar.setArmVoltage(1, 0)));
//    driverController.rightBumper()
//            .whileTrue(new RepeatCommand(new InstantCommand(endEffector::intake)))
//            .whileFalse(new InstantCommand(endEffector::idle));
//    driverController.leftBumper()
//            .whileTrue(new RepeatCommand(new InstantCommand(superstructure::exhaust)))
//            .whileFalse(new InstantCommand(superstructure::home));
//    driverController.rightBumper().whileTrue(new InstantCommand(endEffector::exhaust)).whileFalse(new InstantCommand(endEffector::idle));

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
//    driverController.y()
//            .whileTrue(new SwerveAutoScore())
//            .whileFalse(new SuperstructureHome());
//    driverController.b()
//            .whileTrue(new SwerveAutoDoubleSubstationHigh())
//            .whileFalse(new SuperstructureHome());
//    driverController.a()
//            .whileTrue(new SwerveAutoDoubleSubstationLow())
//            .whileFalse(new SuperstructureHome());
    driverController.rightBumper()
            .whileTrue(new InstantCommand(swerve::zeroGyro));

    /*
     OPERATOR
     */
    operatorController.rightTrigger()
            .whileTrue(new InstantCommand(Superstructure.getInstance()::exhaust))
            .whileFalse(new InstantCommand(Superstructure.getInstance()::idle));
    operatorController.leftTrigger()
            .whileTrue(new InstantCommand(Superstructure.getInstance()::manual))
            .whileFalse(new InstantCommand(Superstructure.getInstance()::idle));
    operatorController.a()
            .whileTrue(new InstantCommand(Superstructure.getInstance()::collectGround))
            .whileFalse(new InstantCommand(Superstructure.getInstance()::home));
    operatorController.b()
            .whileTrue(new InstantCommand(Superstructure.getInstance()::collectMid))
            .whileFalse(new InstantCommand(Superstructure.getInstance()::home));
    operatorController.x()
            .whileTrue(new InstantCommand(Superstructure.getInstance()::objective))
            .whileFalse(new InstantCommand(Superstructure.getInstance()::home));
    operatorController.y()
            .whileTrue(new InstantCommand(Superstructure.getInstance()::setDemandLEDs));

//    operatorController.rightTrigger()
//            .whileTrue(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::exhaust)))
//            .whileFalse(new InstantCommand(Superstructure.getInstance()::idle));
//    operatorController.leftTrigger()
//            .whileTrue(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::manual)))
//            .whileFalse(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::idle)));
//    operatorController.a()
//            .whileTrue(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::collectGround)))
//            .whileFalse(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::home)));
//    operatorController.b()
//            .whileTrue(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::collectMid)))
//            .whileFalse(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::home)));
//    operatorController.x()
//            .whileTrue(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::objective)))
//            .whileFalse(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::home)));
//    operatorController.y()
//            .whileTrue(new RepeatCommand(new InstantCommand(Superstructure.getInstance()::setDemandLEDs)));

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

  private void autoConfig() {
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

    Command balance = new AutoSwerveBalance();
    autoChooser.addOption("Place and Balance Climb", balance);
    canBalanceMap.put(balance, true);
    needPositionMap.put(balance, true);

    Command threeTop = new AutoSwerveThree(true);
    autoChooser.addOption("3 Thing Top", threeTop);
    canBalanceMap.put(threeTop, true);
    needPositionMap.put(threeTop, false);

    Command threeBottom = new AutoSwerveThree(false);
    autoChooser.addOption("3 Thing Bottom", threeBottom);
    canBalanceMap.put(threeBottom, true);
    needPositionMap.put(threeBottom, false);

    Command twoTop = new AutoSwerveTwo(true);
    autoChooser.addOption("2 Thing Top", twoTop);
    canBalanceMap.put(twoTop, false);
    needPositionMap.put(twoTop, false);

    Command twoBottom = new AutoSwerveTwo(false);
    autoChooser.addOption("2 Thing Bottom", twoBottom);
    canBalanceMap.put(twoBottom, false);
    needPositionMap.put(twoBottom, false);

    Command coneTop = new AutoSwervePath(Paths.TOP_CONE, new Objective(8, NodeLevel.HIGH));
    autoChooser.addOption("1 Cone Top", coneTop);
    canBalanceMap.put(coneTop, true);
    needPositionMap.put(coneTop, false);

    Command coneTopPlusOne = new AutoSwervePath(Paths.TOP_CONE_PLUS_ONE, new Objective(8, NodeLevel.HIGH));
    autoChooser.addOption("1 Cone Top + 1", coneTopPlusOne);
    canBalanceMap.put(coneTopPlusOne, true);
    needPositionMap.put(coneTopPlusOne, false);

    Command cubeTop = new AutoSwervePath(Paths.TOP_CUBE, new Objective(7, NodeLevel.HIGH));
    autoChooser.addOption("1 Cube Top", cubeTop);
    canBalanceMap.put(cubeTop, true);
    needPositionMap.put(cubeTop, false);

    Command cubeTopPlusOne = new AutoSwervePath(Paths.TOP_CUBE_PLUS_ONE, new Objective(7, NodeLevel.HIGH));
    autoChooser.addOption("1 Cube Top + 1", cubeTopPlusOne);
    canBalanceMap.put(cubeTopPlusOne, true);
    needPositionMap.put(cubeTopPlusOne, false);

    Command coneBottom = new AutoSwervePath(Paths.BOTTOM_CONE, new Objective(0, NodeLevel.HIGH));
    autoChooser.addOption("1 Cone Bottom", coneBottom);
    canBalanceMap.put(coneBottom, true);
    needPositionMap.put(coneBottom, false);

    Command coneBottomPlusOne = new AutoSwervePath(Paths.BOTTOM_CONE_PLUS_ONE, new Objective(1, NodeLevel.HIGH));
    autoChooser.addOption("1 Cone Bottom + 1", coneBottomPlusOne);
    canBalanceMap.put(coneBottomPlusOne, true);
    needPositionMap.put(coneBottomPlusOne, false);

    Command cubeBottom = new AutoSwervePath(Paths.BOTTOM_CUBE, new Objective(1, NodeLevel.HIGH));
    autoChooser.addOption("1 Cube Bottom", cubeBottom);
    canBalanceMap.put(cubeBottom, true);
    needPositionMap.put(cubeBottom, false);

    Command cubeBottomPlusOne = new AutoSwervePath(Paths.BOTTOM_CUBE_PLUS_ONE, new Objective(1, NodeLevel.HIGH));
    autoChooser.addOption("1 Cube Bottom + 1", cubeBottomPlusOne);
    canBalanceMap.put(cubeBottomPlusOne, true);
    needPositionMap.put(cubeBottomPlusOne, false);

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
    return autoChooser.getSelected();
  }
}