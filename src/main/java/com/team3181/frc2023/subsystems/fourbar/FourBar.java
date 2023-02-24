package com.team3181.frc2023.subsystems.fourbar;


import com.team3181.frc2023.Constants;
import com.team3181.frc2023.Constants.FourBarConstants;
import com.team3181.frc2023.Constants.SwerveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class FourBar extends SubsystemBase {

    private final ArmIO[] armIO;
    private final ArmIOInputsAutoLogged[] armInputs = new ArmIOInputsAutoLogged[]{new ArmIOInputsAutoLogged(), new ArmIOInputsAutoLogged()};

    private final static FourBar INSTANCE = new FourBar(Constants.RobotConstants.SHOULDER, Constants.RobotConstants.ELBOW);

    public static FourBar getInstance() {
        return INSTANCE;
    }

    private FourBar(ArmIO shoulderIO, ArmIO elbowIO) {
        armIO = new ArmIO[]{shoulderIO, elbowIO};
        SmartDashboard.putNumber("number x", 0);
        SmartDashboard.putNumber("number y", 0);
        for(int i = 0; i < 2; i++) {
           armIO[i].updateInputs(armInputs[i]);
        }
    }

    @Override
    public void periodic() {
        for(int i = 0; i < 2; i++) {
            armIO[i].updateInputs(armInputs[i]);
        }
        Logger.getInstance().processInputs("Shoulder", armInputs[0]);
        Logger.getInstance().processInputs("Elbow", armInputs[1]);

        System.out.println(Arrays.toString(solve(new Translation2d(SmartDashboard.getNumber("number x", 0), SmartDashboard.getNumber("number y", 0)), true)));
    }

    public void setArmVoltage(int index, double voltage) {
        armIO[index].setVoltage(voltage);
    }

    /** Converts joint angles to the end effector position. */
    public Translation2d forward() {
        return new Translation2d(
                FourBarConstants.SHOULDER_JOINT_POSITION_X
                        + FourBarConstants.SHOULDER_LENGTH * Math.cos(armInputs[0].armPositionRad)
                        + FourBarConstants.ELBOW_LENGTH * Math.cos(armInputs[0].armPositionRad + armInputs[1].armPositionRad),
                FourBarConstants.SHOULDER_JOINT_POSITION_Y
                        + FourBarConstants.SHOULDER_LENGTH * Math.sin(armInputs[0].armPositionRad)
                        + FourBarConstants.ELBOW_LENGTH * Math.sin(armInputs[0].armPositionRad + armInputs[1].armPositionRad)
        );
    }

    public Rotation2d[] solve(Translation2d position, boolean bumper) {
        double newX = position.getX() + SwerveConstants.X_LENGTH_METERS / 2;
        double newY = position.getY() - FourBarConstants.CHASSIS_TO_ARM - FourBarConstants.WHEEL_TO_CHASSIS;
        Translation2d updatedPos = bumper ? new Translation2d(newX, newY) : position;

        double rotElbow;
        double rotShoulder;

        if (updatedPos.getY() > 0) {
            rotElbow = -1 * Math.acos((Math.pow(updatedPos.getX(), 2) + Math.pow(updatedPos.getY(), 2) - Math.pow(FourBarConstants.SHOULDER_LENGTH, 2) - Math.pow(FourBarConstants.ELBOW_LENGTH, 2))/(2 * FourBarConstants.SHOULDER_LENGTH * FourBarConstants.ELBOW_LENGTH));
        }
        else {
            rotElbow = 1 * Math.acos((Math.pow(updatedPos.getX(), 2) + Math.pow(updatedPos.getY(), 2) - Math.pow(FourBarConstants.SHOULDER_LENGTH, 2) - Math.pow(FourBarConstants.ELBOW_LENGTH, 2))/(2 * FourBarConstants.SHOULDER_LENGTH * FourBarConstants.ELBOW_LENGTH));
        }
        rotShoulder = Math.atan(updatedPos.getY()/updatedPos.getX()) - Math.atan((FourBarConstants.ELBOW_LENGTH * Math.sin(rotElbow))/(FourBarConstants.SHOULDER_LENGTH + FourBarConstants.ELBOW_LENGTH * Math.cos(rotElbow)));

//        if (Double.isNaN(rotElbow) || Double.isNaN(rotShoulder) || rotElbow > FourBarConstants.ELBOW_MAX.getRadians() || rotElbow < FourBarConstants.ELBOW_MIN.getRadians() || rotShoulder > FourBarConstants.SHOULDER_MAX.getRadians() || rotShoulder < FourBarConstants.SHOULDER_MIN.getRadians()) {
//            rotElbow *= -1;
//            rotShoulder = Math.atan(updatedPos.getY()/updatedPos.getX()) - Math.atan((FourBarConstants.ELBOW_LENGTH * Math.sin(rotElbow))/(FourBarConstants.SHOULDER_LENGTH + FourBarConstants.ELBOW_LENGTH * Math.cos(rotElbow)));
//
//            if (Double.isNaN(rotElbow) || Double.isNaN(rotShoulder) || rotElbow > FourBarConstants.ELBOW_MAX.getRadians() || rotElbow < FourBarConstants.ELBOW_MIN.getRadians() || rotShoulder > FourBarConstants.SHOULDER_MAX.getRadians() || rotShoulder < FourBarConstants.SHOULDER_MIN.getRadians()) {
//                return new Rotation2d[] {null, null};
//            }
//        }

        return new Rotation2d[] {Rotation2d.fromRadians(rotShoulder), Rotation2d.fromRadians(rotElbow)};
    }
}