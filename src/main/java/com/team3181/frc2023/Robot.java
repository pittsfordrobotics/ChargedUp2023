// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3181.frc2023;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.lib.util.Alert;
import com.team3181.lib.util.Alert.AlertType;
import com.team3181.lib.util.PIDTuner;
import com.team3181.lib.controller.BetterXboxController;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final Alert logReceiverQueueAlert = new Alert("Logging queue is full. Data will NOT be logged.", AlertType.ERROR);
  private final Alert driverControllerAlert = new Alert("Driver Controller is NOT detected!", AlertType.ERROR);
  private final Alert operatorControllerAlert = new Alert("Operator Controller is NOT detected!", AlertType.ERROR);

  private final Timer disabledTimer = new Timer();
  private final boolean stopped = false;

  private RobotContainer robotContainer;

  @Override
  public void robotInit() {
    Logger logger = Logger.getInstance();
    setUseTiming(true);
    logger.recordMetadata("PIDTuner", Boolean.toString(RobotConstants.PID_TUNER_ENABLED));
    logger.recordMetadata("Demo Mode", Boolean.toString(RobotConstants.DEMO_MODE));
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

//    Logger startup
    logger.addDataReceiver(new NT4Publisher());
    if (RobotBase.isReal()) {
      logger.addDataReceiver(new WPILOGWriter(RobotConstants.LOGGING_PATH));
      LoggedPowerDistribution.getInstance();
    }
    if (RobotConstants.LOGGING_ENABLED) {
      logger.start();
    }
    PIDTuner.enable(RobotConstants.PID_TUNER_ENABLED);

    DriverStation.silenceJoystickConnectionWarning(true);
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    CommandScheduler.getInstance().run();

    // Log scheduled commands
    Logger.getInstance().recordOutput("ActiveCommands/Scheduler",
            NetworkTableInstance.getDefault()
                    .getEntry("/LiveWindow/Ungrouped/Scheduler/Names")
                    .getStringArray(new String[] {}));

    logReceiverQueueAlert.set(Logger.getInstance().getReceiverQueueFault());

    new BetterXboxController(0, BetterXboxController.Humans.DRIVER);
    new BetterXboxController(1, BetterXboxController.Humans.OPERATOR);

    driverControllerAlert.set(!DriverStation.isJoystickConnected(0));
    operatorControllerAlert.set(!DriverStation.isJoystickConnected(1));
    Threads.setCurrentThreadPriority(false, 10);
  }

  @Override
  public void disabledInit() {
    Swerve.getInstance().stopMotors();
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    if (Swerve.getInstance().isStopped() && disabledTimer.hasElapsed(3)) {
      Swerve.getInstance().setCoastMode();
      disabledTimer.stop();
    }
  }

  @Override
  public void autonomousInit() {
    Swerve.getInstance().setBrakeMode();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Swerve.getInstance().setBrakeMode();
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
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