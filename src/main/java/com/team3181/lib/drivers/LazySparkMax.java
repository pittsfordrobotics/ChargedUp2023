package com.team3181.lib.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.lib.util.Alert;
import com.team3181.lib.util.Alert.AlertType;

/**
 * This is a thick wrapper for CANSparkMax because I am lazy
 */
public class LazySparkMax extends CANSparkMax {
    private int errors = 1;
    private int attempts = -1;

    /**
     * Lazy Spark Max
     * @param port port of CAN ID of CANSparkMax
     * @param mode Brake or Coast
     * @param currentLimit 0-40 for 550 / 0-80 for NEO
     * @param inverted Inverted?
     */
    public LazySparkMax(int port, IdleMode mode, int currentLimit, boolean inverted) {
        super(port, MotorType.kBrushless);
        while (errors > 0 && ++attempts <= 5) {
            if (attempts > 0) {
                DriverStation.reportWarning("SparkMAX " + port + " FAILED to initialize. Reinitializing attempt " + attempts, false);
            }
            errors = 0;
            errors += check(restoreFactoryDefaults());
            setInverted(inverted);
            errors += check(setIdleMode(mode));
            errors += check(enableVoltageCompensation(12));
            errors += check(getEncoder().setPosition(0));
            errors += check(setSmartCurrentLimit(currentLimit));
            errors += check(burnFlash());
        }
        if (errors > 0) {
            new Alert("SparkMAX Errors", RobotConstants.SPARKMAX_HASHMAP.get(port) + " FAILED to initialize (" + port + ").", AlertType.ERROR).set(true);
        }
    }

    /**
     * Lazy Spark Max not inverted
     * @param port port of CAN ID of CANSparkMax
     * @param mode Brake or Coast
     * @param currentLimit 0-40 for 550 / 0-80 for NEO
     */
    public LazySparkMax(int port, IdleMode mode, int currentLimit) {
        this(port, mode, currentLimit, false);
    }

    /**
     * Lazy Spark Max
     * @param port port of CAN ID of CANSparkMax
     * @param mode Brake or Coast
     * @param currentLimit 0-30 for 550 / 0-80 for NEO
     * @param leader SparkMax to follow
     * @param inverted Whether to follow the leader inverted
     */
    public LazySparkMax(int port, IdleMode mode, int currentLimit, CANSparkMax leader, boolean inverted) {
        super(port, MotorType.kBrushless);
        while (errors > 0 && ++attempts <= 5) {
            if (attempts > 0) {
                DriverStation.reportWarning("SparkMAX " + port + " FAILED to initialize. Reinitializing attempt " + attempts, false);
            }
            errors = 0;
            errors += check(restoreFactoryDefaults());
            errors += check(setIdleMode(mode));
            errors += check(enableVoltageCompensation(12));
            errors += check(getEncoder().setPosition(0));
            errors += check(setSmartCurrentLimit(currentLimit));
            errors += check(follow(leader, inverted));
            errors += check(burnFlash());
        }
        if (errors > 0) {
            new Alert("SparkMAX Errors", RobotConstants.SPARKMAX_HASHMAP.get(port) + " FAILED to initialize (" + port + ").", AlertType.ERROR).set(true);
        }
    }

    /**
     * Lazy Spark Max
     * @param port port of CAN ID of CANSparkMax
     * @param mode Brake or Coast
     * @param currentLimit 0-40 for 550 / 0-80 for NEO
     * @param leader SparkMax to follow
     */
    public LazySparkMax(int port, IdleMode mode, int currentLimit, CANSparkMax leader) {
        this(port, mode, currentLimit, leader, false);
    }

    public double getAppliedVoltage() {
        return RobotController.getBatteryVoltage() * getAppliedOutput();
    }

    /**
     * Used for checking RevLib functions
     * @return 1 for error, 0 for no error
     */
    private int check(REVLibError err) {
        return err == REVLibError.kOk ? 0 : 1;
    }
}