/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team3181.frc2023.commands;

import com.team3181.frc2023.subsystems.tank.Tank;
import com.team3181.lib.controller.BetterXboxController;
import com.team3181.lib.controller.BetterXboxController.Humans;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TankXbox extends CommandBase {
    private final Tank drive = Tank.getInstance();
    private final BetterXboxController driverController = BetterXboxController.getController(Humans.DRIVER);

    public TankXbox() {
        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        drive.driveCurve(MathUtil.applyDeadband(-driverController.getLeftY(), 0.15), MathUtil.applyDeadband(-driverController.getRightX(), 0.15));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}