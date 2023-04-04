// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3181.frc2023;

import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.subsystems.fourbar.FourBar;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.lib.controller.BetterXboxController;
import com.team3181.lib.drivers.LazySparkMax;
import com.team3181.lib.util.Alert;
import com.team3181.lib.util.Alert.AlertType;
import com.team3181.lib.util.PIDTuner;
import com.team3181.lib.util.VirtualSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final Alert logReceiverQueueAlert = new Alert("Logging queue is full. Data will NOT be logged.", AlertType.ERROR);
  private final Alert driverControllerAlert = new Alert("Driver Controller is NOT detected!", AlertType.ERROR);
  private final Alert operatorControllerAlert = new Alert("Operator Controller is NOT detected!", AlertType.ERROR);
  private final Alert lowBatteryAlert = new Alert("Battery is at a LOW voltage! The battery MUST be replaced before playing a match!", AlertType.WARNING);

  private final Timer disabledTimer = new Timer();
  private final Timer garbageCollector = new Timer();
  public static final ShuffleboardTab pitTab = Shuffleboard.getTab("Pit");
  private boolean stopped = false;

  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    Logger logger = Logger.getInstance();
    setUseTiming(true);
    logger.recordMetadata("PIDTuner", Boolean.toString(RobotConstants.PID_TUNER_ENABLED));
    logger.recordMetadata("Tank", Boolean.toString(RobotConstants.IS_TANK));
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0 -> logger.recordMetadata("GitDirty", "All changes committed");
      case 1 -> logger.recordMetadata("GitDirty", "Uncommitted changes");
      default -> logger.recordMetadata("GitDirty", "Unknown");
    }

//    Logger startup
    logger.addDataReceiver(new NT4Publisher());
    if (RobotBase.isReal()) {
      logger.addDataReceiver(new WPILOGWriter(RobotConstants.LOGGING_PATH));
      LoggedPowerDistribution.getInstance(0, ModuleType.kRev);
    }
    else if (RobotConstants.REPLAY_ENABLED) {
      String path = LogFileUtil.findReplayLog();
      logger.setReplaySource(new WPILOGReader(path));
      logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_replayed")));
    }
    if (RobotConstants.LOGGING_ENABLED) {
      logger.start();
    }
    PIDTuner.enable(RobotConstants.PID_TUNER_ENABLED);

    DriverStation.silenceJoystickConnectionWarning(true);
    LiveWindow.disableAllTelemetry();
    LiveWindow.setEnabled(false);
    robotContainer = new RobotContainer();
    Swerve.getInstance().setCoastMode();
    garbageCollector.start();

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
              String name = command.getName();
              int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
              commandCounts.put(name, count);
              Logger.getInstance()
                      .recordOutput(
                              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
              Logger.getInstance().recordOutput("CommandsAll/" + name, count > 0);
            };
    CommandScheduler.getInstance().onCommandInitialize((Command command) -> {
                      logCommandFunction.accept(command, true);
                    });
    CommandScheduler.getInstance().onCommandFinish((Command command) -> {
                      logCommandFunction.accept(command, false);
                    });
    CommandScheduler.getInstance().onCommandInterrupt((Command command) -> {
                      logCommandFunction.accept(command, false);
                    });

    new BetterXboxController(0, BetterXboxController.Humans.DRIVER);
    new BetterXboxController(1, BetterXboxController.Humans.OPERATOR);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    VirtualSubsystem.periodicAll();

    logReceiverQueueAlert.set(Logger.getInstance().getReceiverQueueFault());
    // in MBs
    Logger.getInstance().recordOutput("Memory/Usage", (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / 1024.0 / 1024.0);
    Logger.getInstance().recordOutput("Memory/Free", (Runtime.getRuntime().freeMemory()) / 1024.0 / 1024.0);
    Logger.getInstance().recordOutput("Memory/Total", (Runtime.getRuntime().totalMemory()) / 1024.0 / 1024.0);

    driverControllerAlert.set(!DriverStation.isJoystickConnected(0));
    operatorControllerAlert.set(!DriverStation.isJoystickConnected(1));
    LazySparkMax.checkAlive();

    SmartDashboard.putBoolean("Can Balance", robotContainer.canBalance());
    SmartDashboard.putBoolean("Need Position", robotContainer.needPosition());

    // try to fix mem problems
//    if (garbageCollector.hasElapsed(5)) {
//      System.gc();
//      garbageCollector.restart();
//    }
  }

  @Override
  public void disabledInit() {
    disabledTimer.restart();
    stopped = false;
  }

  @Override
  public void disabledPeriodic() {
    lowBatteryAlert.set(RobotController.getBatteryVoltage() < 12.2);
    if (disabledTimer.hasElapsed(5) && !stopped) {
      Swerve.getInstance().setCoastMode();
      disabledTimer.stop();
      stopped = true;
    }
  }

  @Override
  public void autonomousInit() {
    lowBatteryAlert.set(false);
    Swerve.getInstance().setBrakeMode();
    FourBar.getInstance().brake();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    lowBatteryAlert.set(false);
    Swerve.getInstance().setBrakeMode();
    FourBar.getInstance().brake();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
      autonomousCommand = null;
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}