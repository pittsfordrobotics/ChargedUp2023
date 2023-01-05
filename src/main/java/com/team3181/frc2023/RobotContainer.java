// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3181.frc2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.commands.Auto5Ball;
import com.team3181.frc2023.commands.SwerveDriveFieldXbox;
import com.team3181.frc2023.commands.SwervePathing;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.lib.controller.BetterXboxController;

public class RobotContainer {
  private final BetterXboxController driverController = new BetterXboxController(0, BetterXboxController.Humans.DRIVER);
  private final BetterXboxController operatorController = new BetterXboxController(1, BetterXboxController.Humans.OPERATOR);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final SendableChooser<Pose2d> positionChooser = new SendableChooser<>();

  public RobotContainer() {
    autoConfig();
    driverDashboardSetup();

    if (RobotConstants.DEMO_MODE) {
      demoButtons();
    }
    else {
//      competitionButtons();
      testButtons();
    }

    Swerve.getInstance().setDefaultCommand(new SwerveDriveFieldXbox());
  }

  private void driverDashboardSetup() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  }

  private void testButtons() {
  }

  private void competitionButtons() {}

  private void demoButtons() {}

  private void autoConfig() {
    autoChooser.setDefaultOption("No auto", new WaitCommand(0));
    autoChooser.addOption("Test", new SwervePathing(Paths.TEST, true));
    autoChooser.addOption("5 ball", new Auto5Ball());

    SmartDashboard.putData("Auto Command", autoChooser);
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}