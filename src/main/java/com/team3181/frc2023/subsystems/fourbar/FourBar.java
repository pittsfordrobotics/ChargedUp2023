package com.team3181.frc2023.subsystems.fourbar;


import com.team3181.frc2023.Constants;
import com.team3181.frc2023.Constants.EndEffectorConstants;
import com.team3181.frc2023.Constants.FourBarConstants;
import com.team3181.frc2023.Constants.SwerveConstants;
import com.team3181.lib.util.Alert;
import com.team3181.lib.util.Alert.AlertType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class FourBar extends SubsystemBase {
    private final ArmIO[] armIO;
    private final ArmIOInputsAutoLogged[] inputs = new ArmIOInputsAutoLogged[]{new ArmIOInputsAutoLogged(), new ArmIOInputsAutoLogged()};

    private final PIDController shoulderPID = new PIDController(FourBarConstants.SHOULDER_P, FourBarConstants.SHOULDER_I, FourBarConstants.SHOULDER_D);
    private final PIDController elbowPID = new PIDController(FourBarConstants.ELBOW_P, FourBarConstants.ELBOW_I, FourBarConstants.ELBOW_D);
    private final Alert shoulderTooLow = new Alert("Shoulder needs to be moved forward! THIS WILL BREAK ALOT OF THINGS!", AlertType.ERROR);

    private final static FourBar INSTANCE = new FourBar(Constants.RobotConstants.SHOULDER, Constants.RobotConstants.ELBOW);

    public static FourBar getInstance() {
        return INSTANCE;
    }

    private FourBar(ArmIO shoulderIO, ArmIO elbowIO) {
        armIO = new ArmIO[]{shoulderIO, elbowIO};
        SmartDashboard.putNumber("shoulder", 0);
        SmartDashboard.putNumber("elbow", 0);
        shoulderPID.setTolerance(FourBarConstants.PID_TOLERANCE);
        elbowPID.setTolerance(FourBarConstants.PID_TOLERANCE);

        for(int i = 0; i < 2; i++) {
           armIO[i].updateInputs(inputs[i]);
        }
    }

    @Override
    public void periodic() {
        for(int i = 0; i < 2; i++) {
            armIO[i].updateInputs(inputs[i]);
        }
        Logger.getInstance().processInputs("Shoulder", inputs[0]);
        Logger.getInstance().processInputs("Elbow", inputs[1]);

        shoulderTooLow.set(DriverStation.isDisabled() && inputs[0].armPositionRad > 0);

        System.out.println(Arrays.toString(
                solve(
                        new Translation2d(
                                SmartDashboard.getNumber("shoulder", 0),
                                SmartDashboard.getNumber("elbow", 0)), true, true)));
//        setRotations(new Rotation2d[] {
//                Rotation2d.fromRadians(SmartDashboard.getNumber("shoulder", 0)),
//                Rotation2d.fromRadians(SmartDashboard.getNumber("elbow", 0))
//        });

    }

    public void hold() {
        Vector<N2> pos = new Vector<N2>(VecBuilder.fill(inputs[0].armPositionRad, inputs[1].armPositionRad));
        Vector<N2> ff = ArmDynamics. getInstance().feedforward(pos);
        setArmVoltage(0, ff.get(0, 0));
        setArmVoltage(1, ff.get(1, 0));
    }

    public void setArmVoltage(int index, double voltage) {
        Vector<N2> pos = new Vector<N2>(VecBuilder.fill(inputs[0].armPositionRad, inputs[1].armPositionRad));
        Vector<N2> ff = ArmDynamics. getInstance().feedforward(pos);
        if (index == 0) {
            armIO[index].setVoltage(voltage + ff.get(0, 0));
        }
        else if (index == 1) {
            armIO[index].setVoltage(voltage + ff.get(1, 0));
        }
    }

    public void setRotations(Rotation2d[] rotations) {
        Boolean[] illegal = checkIllegal(rotations);
        Vector<N2> pos = new Vector<N2>(VecBuilder.fill(inputs[0].armPositionRad, inputs[1].armPositionRad));
        Vector<N2> ff = ArmDynamics.getInstance().feedforward(pos);
//        if (!illegal[0]) {
            shoulderPID.setSetpoint(rotations[0].getRadians());
            armIO[0].setVoltage(MathUtil.clamp(shoulderPID.calculate(inputs[0].armPositionRad), -FourBarConstants.PID_CLAMP_VOLTAGE, FourBarConstants.PID_CLAMP_VOLTAGE) + ff.get(0, 0));
//        if (!illegal[1]) {
            elbowPID.setSetpoint(rotations[1].getRadians());
            armIO[1].setVoltage(MathUtil.clamp(elbowPID.calculate(inputs[1].armPositionRad - inputs[0].armPositionRad), -FourBarConstants.PID_CLAMP_VOLTAGE, FourBarConstants.PID_CLAMP_VOLTAGE) + ff.get(1, 0));
//        }
    }

    public boolean atSetpoint() {
        return shoulderPID.atSetpoint() && elbowPID.atSetpoint();
    }

    /** Converts joint angles to the end effector position. */
    public Translation2d forward() {
        return new Translation2d(
                FourBarConstants.SHOULDER_JOINT_POSITION_X
                        + FourBarConstants.SHOULDER_LENGTH * Math.cos(inputs[0].armPositionRad)
                        + FourBarConstants.ELBOW_LENGTH * Math.cos(inputs[0].armPositionRad + inputs[1].armPositionRad),
                FourBarConstants.SHOULDER_JOINT_POSITION_Y
                        + FourBarConstants.SHOULDER_LENGTH * Math.sin(inputs[0].armPositionRad)
                        + FourBarConstants.ELBOW_LENGTH * Math.sin(inputs[0].armPositionRad + inputs[1].armPositionRad)
        );
    }

    public Translation2d forward(Rotation2d[] rotations) {
        return new Translation2d(
                FourBarConstants.SHOULDER_JOINT_POSITION_X
                        + FourBarConstants.SHOULDER_LENGTH * Math.cos(rotations[0].getRadians())
                        + FourBarConstants.ELBOW_LENGTH * Math.cos(rotations[0].getRadians() + rotations[1].getRadians()),
                FourBarConstants.SHOULDER_JOINT_POSITION_Y
                        + FourBarConstants.SHOULDER_LENGTH * Math.sin(rotations[0].getRadians())
                        + FourBarConstants.ELBOW_LENGTH * Math.sin(rotations[0].getRadians() + rotations[1].getRadians())
        );
    }

    public Translation2d forwardWithEndEffector(Rotation2d[] rotations) {
        return new Translation2d(
                FourBarConstants.SHOULDER_JOINT_POSITION_X
                        + FourBarConstants.SHOULDER_LENGTH * Math.cos(rotations[0].getRadians())
                        + (FourBarConstants.ELBOW_LENGTH + EndEffectorConstants.LENGTH) * Math.cos(rotations[0].getRadians() + rotations[1].getRadians()),
                FourBarConstants.SHOULDER_JOINT_POSITION_Y
                        + FourBarConstants.SHOULDER_LENGTH * Math.sin(rotations[0].getRadians())
                        + (FourBarConstants.ELBOW_LENGTH + EndEffectorConstants.LENGTH) * Math.sin(rotations[0].getRadians() + rotations[1].getRadians())
        );
    }

    public Rotation2d[] solve(Translation2d position, boolean cone, boolean bumper) {
        double newX = position.getX() + SwerveConstants.X_LENGTH_METERS / 2;
        double newY = position.getY() - FourBarConstants.CHASSIS_TO_ARM - FourBarConstants.WHEEL_TO_CHASSIS;
        Translation2d updatedPos = bumper ? new Translation2d(newX, newY) : position;

        double rotElbow;
        double rotShoulder;

        if (cone) {
            rotElbow = -1 * Math.acos((Math.pow(updatedPos.getX(), 2) + Math.pow(updatedPos.getY(), 2) - Math.pow(FourBarConstants.SHOULDER_LENGTH, 2) - Math.pow(FourBarConstants.ELBOW_LENGTH, 2))/(2 * FourBarConstants.SHOULDER_LENGTH * FourBarConstants.ELBOW_LENGTH));
        }
        else {
            rotElbow = 1 * Math.acos((Math.pow(updatedPos.getX(), 2) + Math.pow(updatedPos.getY(), 2) - Math.pow(FourBarConstants.SHOULDER_LENGTH, 2) - Math.pow(FourBarConstants.ELBOW_LENGTH, 2))/(2 * FourBarConstants.SHOULDER_LENGTH * FourBarConstants.ELBOW_LENGTH));
        }
        rotShoulder = Math.atan(updatedPos.getY()/updatedPos.getX()) - Math.atan((FourBarConstants.ELBOW_LENGTH * Math.sin(rotElbow))/(FourBarConstants.SHOULDER_LENGTH + FourBarConstants.ELBOW_LENGTH * Math.cos(rotElbow)));
        System.out.println(rotShoulder);
        System.out.println(rotElbow);
        if (Double.isNaN(rotElbow) || Double.isNaN(rotShoulder) || rotElbow > FourBarConstants.ELBOW_MAX.getRadians() || rotElbow < FourBarConstants.ELBOW_MIN.getRadians() || rotShoulder > FourBarConstants.SHOULDER_MAX.getRadians() || rotShoulder < FourBarConstants.SHOULDER_MIN.getRadians()) {
            rotElbow *= -1;
            rotShoulder = Math.atan(updatedPos.getY()/updatedPos.getX()) - Math.atan((FourBarConstants.ELBOW_LENGTH * Math.sin(rotElbow))/(FourBarConstants.SHOULDER_LENGTH + FourBarConstants.ELBOW_LENGTH * Math.cos(rotElbow)));
            if (Double.isNaN(rotElbow) || Double.isNaN(rotShoulder) || rotElbow > FourBarConstants.ELBOW_MAX.getRadians() || rotElbow < FourBarConstants.ELBOW_MIN.getRadians() || rotShoulder > FourBarConstants.SHOULDER_MAX.getRadians() || rotShoulder < FourBarConstants.SHOULDER_MIN.getRadians()) {
                return new Rotation2d[] {null, null};
            }
        }
        return new Rotation2d[] {Rotation2d.fromRadians(rotShoulder), Rotation2d.fromRadians(rotElbow)};
    }

    /**
     *
     * @param rotations
     * @return whether it is illegal to move for {shoulder, elbow}
     */
    private Boolean[] checkIllegal(Rotation2d[] rotations) {
        Rotation2d[] shoulderConstant = new Rotation2d[] {Rotation2d.fromRadians(inputs[0].armPositionRad), rotations[1]};
        Rotation2d[] elbowConstant = new Rotation2d[] {rotations[0], Rotation2d.fromRadians(inputs[1].armPositionRad)};

        Translation2d endPositionShoulder = forwardWithEndEffector(elbowConstant);
        Translation2d endPositionElbow = forwardWithEndEffector(shoulderConstant);

        return new Boolean[] {checkIllegal(endPositionShoulder), checkIllegal(endPositionElbow)};
    }

    /**
     *
     * @param endEffectorPos
     * @return whether it is illegal to move there
     */
    private boolean checkIllegal(Translation2d endEffectorPos) {
        if (endEffectorPos.getX() < 0 && endEffectorPos.getY() < FourBarConstants.WHEEL_TO_CHASSIS) {
            return true;
        }
        else if (endEffectorPos.getY() < 0) {
            return true;
        }
        return false;
    }
}