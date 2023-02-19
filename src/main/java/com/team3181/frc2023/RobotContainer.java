// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3181.frc2023;

import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.commands.AutoCollectAndGo;
import com.team3181.frc2023.commands.DropClimb;
import com.team3181.frc2023.commands.SwerveDriveFieldXbox;
import com.team3181.frc2023.commands.TankXbox;
import com.team3181.frc2023.subsystems.endeffector.EndEffector;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.frc2023.subsystems.tank.Tank;
import com.team3181.frc2023.subsystems.vision.Vision;
import com.team3181.lib.controller.BetterXboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {
  private final Swerve swerve = Swerve.getInstance();
  private final EndEffector endEffector = EndEffector.getInstance();
  private final Vision vision = Vision.getInstance();

  private final BetterXboxController driverController = new BetterXboxController(0, BetterXboxController.Humans.DRIVER);
  private final BetterXboxController operatorController = new BetterXboxController(1, BetterXboxController.Humans.OPERATOR);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final SendableChooser<Pose2d> positionChooser = new SendableChooser<>();

  public RobotContainer() {
    autoConfig();
    driverDashboardSetup();

//      competitionButtons();
      testButtons();

    if (!RobotConstants.IS_TANK) swerve.setDefaultCommand(new SwerveDriveFieldXbox());
    if (RobotConstants.IS_TANK) Tank.getInstance().setDefaultCommand(new TankXbox());
  }

  private void driverDashboardSetup() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  }

  private void testButtons() {
//    driverController.a().onTrue(new InstantCommand(endEffector::intake)).onFalse(new InstantCommand(endEffector::idle));
//    driverController.x().onTrue(new InstantCommand(endEffector::exhaust)).onFalse(new InstantCommand(endEffector::idle));
    driverController.start().whileTrue(new InstantCommand(swerve::zeroGyro));
    driverController.back().whileTrue(new InstantCommand(swerve::driveX));
//    driverController.a().whileTrue(new SwervePathing(Paths.TEST_ON_THE_FLY, false));
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