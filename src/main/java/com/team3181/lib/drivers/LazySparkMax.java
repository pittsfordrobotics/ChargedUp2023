package com.team3181.lib.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.lib.util.Alert;
import com.team3181.lib.util.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

/**
 * This is a thick wrapper for CANSparkMax because I am lazy
 */
public class LazySparkMax extends CANSparkMax {
    private int errors = 1;
    private int attempts = -1;
    private static final ArrayList<LazySparkMax> sparkMaxes = new ArrayList<>();
    private static final ArrayList<Alert> alerts = new ArrayList<>();

    /**
     * Lazy Spark Max
     * @param port port of CAN ID of CANSparkMax
     * @param mode Brake or Coast
     * @param currentLimit 0-30 for 550 / 0-80 for NEO
     * @param inverted Inverted?
     */
    public LazySparkMax(int port, IdleMode mode, int currentLimit, boolean inverted, boolean burn) {
        super(port, MotorType.kBrushless);
        while (errors > 0 && ++attempts <= 5) {
            if (attempts > 0) {
                DriverStation.reportWarning("SparkMax " + port + "FAILED to initialize. Reinitializing attempt " + attempts, false);
                Logger.getInstance().recordOutput("SparkMaxes/" + RobotConstants.SPARKMAX_HASHMAP.get(port) + port, "Reinitializing attempt " + attempts);
            }
            errors = 0;
            errors += check(restoreFactoryDefaults());
            setInverted(inverted);
            errors += check(setIdleMode(mode));
            errors += check(enableVoltageCompensation(12));
            errors += check(getEncoder().setPosition(0));
            errors += check(setSmartCurrentLimit(currentLimit));
            if (burn) errors += check(burnFlash());
        }
        if (errors > 0) {
            Logger.getInstance().recordOutput("SparkMaxes/" + RobotConstants.SPARKMAX_HASHMAP.get(port) + port, "INITIALIZE_ERROR");
            new Alert("SparkMaxes", RobotConstants.SPARKMAX_HASHMAP.get(port) + " FAILED to initialize (" + port + ")!", AlertType.ERROR).set(true);
        }
        else {
            sparkMaxes.add(this);
            alerts.add(new Alert("SparkMaxes", "Error", AlertType.ERROR));
        }
    }

    /**
     * Lazy Spark Max not inverted
     * @param port port of CAN ID of CANSparkMax
     * @param mode Brake or Coast
     * @param currentLimit 0-30 for 550 / 0-80 for NEO
     */
    public LazySparkMax(int port, IdleMode mode, int currentLimit) {
        this(port, mode, currentLimit, false, true);
    }

    /**
     * Lazy Spark Max
     * @param port port of CAN ID of CANSparkMax
     * @param mode Brake or Coast
     * @param currentLimit 0-30 for 550 / 0-80 for NEO
     * @param leader SparkMax to follow
     * @param inverted Whether to follow the leader inverted
     */
    public LazySparkMax(int port, IdleMode mode, int currentLimit, CANSparkMax leader, boolean inverted, boolean burn) {
        super(port, MotorType.kBrushless);
        while (errors > 0 && ++attempts <= 5) {
            if (attempts > 0) {
                DriverStation.reportWarning("SparkMax " + port + "FAILED to initialize. Reinitializing attempt " + attempts, false);
                Logger.getInstance().recordOutput("SparkMaxes/" + RobotConstants.SPARKMAX_HASHMAP.get(port) + port, "Reinitializing attempt " + attempts);
            }
            errors = 0;
            errors += check(restoreFactoryDefaults());
            errors += check(setIdleMode(mode));
            errors += check(enableVoltageCompensation(12));
            errors += check(getEncoder().setPosition(0));
            errors += check(setSmartCurrentLimit(currentLimit));
            errors += check(follow(leader, inverted));
            if (burn) errors += check(burnFlash());
        }
        if (errors > 0) {
            Logger.getInstance().recordOutput("SparkMaxes/" + RobotConstants.SPARKMAX_HASHMAP.get(port) + port, "INITIALIZE_ERROR");
            new Alert("SparkMaxes",RobotConstants.SPARKMAX_HASHMAP.get(port) + " FAILED to initialize (" + port + ")!", AlertType.ERROR).set(true);
        }
        else {
            sparkMaxes.add(this);
            alerts.add(new Alert("SparkMaxes", "Error", AlertType.ERROR));
        }
    }

    /**
     * Lazy Spark Max
     * @param port port of CAN ID of CANSparkMax
     * @param mode Brake or Coast
     * @param currentLimit 0-30 for 550 / 0-80 for NEO
     * @param leader SparkMax to follow
     */
    public LazySparkMax(int port, IdleMode mode, int currentLimit, CANSparkMax leader) {
        this(port, mode, currentLimit, leader, false, true);
    }

    public static void checkAlive() {
        for (int i = 0; i < sparkMaxes.size(); i++) {
            if (LazySparkMax.check(sparkMaxes.get(i).getLastError()) != 0) {
                Logger.getInstance().recordOutput("SparkMaxes/" + RobotConstants.SPARKMAX_HASHMAP.get(sparkMaxes.get(i).getDeviceId()) + sparkMaxes.get(i).getDeviceId(), sparkMaxes.get(i).getLastError().toString());
                alerts.get(i).setText(RobotConstants.SPARKMAX_HASHMAP.get(sparkMaxes.get(i).getDeviceId()) + " (" + sparkMaxes.get(i).getDeviceId() + ") PROBLEMED with " + sparkMaxes.get(i).getLastError().toString() + " !");
                alerts.get(i).set(true);
            }
            else {
                alerts.get(i).set(false);
            }
        }
    }

    /**
     * Used for checking RevLib functions
     * @return 1 for error, 0 for no error
     */
    private static int check(REVLibError err) {
        return err == REVLibError.kOk ? 0 : 1;
    }
}