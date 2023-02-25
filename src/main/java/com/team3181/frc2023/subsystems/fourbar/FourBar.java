package com.team3181.frc2023.subsystems.fourbar;


import com.team3181.frc2023.Constants;
import com.team3181.frc2023.Constants.FourBarConstants;
import com.team3181.frc2023.Constants.SwerveConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FourBar extends SubsystemBase {
    private final ArmIO[] armIO;
    private final ArmIOInputsAutoLogged[] inputs = new ArmIOInputsAutoLogged[]{new ArmIOInputsAutoLogged(), new ArmIOInputsAutoLogged()};

    private final PIDController shoulderPID = new PIDController(FourBarConstants.SHOULDER_P, FourBarConstants.SHOULDER_I, FourBarConstants.SHOULDER_D);
    private final PIDController elbowPID = new PIDController(FourBarConstants.ELBOW_P, FourBarConstants.ELBOW_I, FourBarConstants.ELBOW_D);

    private final static FourBar INSTANCE = new FourBar(Constants.RobotConstants.SHOULDER, Constants.RobotConstants.ELBOW);

    public static FourBar getInstance() {
        return INSTANCE;
    }

    private FourBar(ArmIO shoulderIO, ArmIO elbowIO) {
        armIO = new ArmIO[]{shoulderIO, elbowIO};
        SmartDashboard.putNumber("number x", 0);
        SmartDashboard.putNumber("number y", 0);

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

        System.out.println(forward(new Rotation2d[] {
                Rotation2d.fromRadians(SmartDashboard.getNumber("number x", 0)),
                Rotation2d.fromRadians(SmartDashboard.getNumber("number y", 0))
        }).toString());
    }

    public void setArmVoltage(int index, double voltage) {
        armIO[index].setVoltage(voltage);
    }

    public void setRotations(Rotation2d[] rotations) {
        shoulderPID.setSetpoint(rotations[0].getRadians());
        armIO[0].setVoltage(shoulderPID.calculate(inputs[0].armPositionRad));

        elbowPID.setSetpoint(rotations[1].getRadians());
        armIO[1].setVoltage(shoulderPID.calculate(inputs[1].armPositionRad));
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
                        + (FourBarConstants.ELBOW_LENGTH) * Math.cos(rotations[0].getRadians() + rotations[1].getRadians()),
                FourBarConstants.SHOULDER_JOINT_POSITION_Y
                        + FourBarConstants.SHOULDER_LENGTH * Math.sin(rotations[0].getRadians())
                        + FourBarConstants.ELBOW_LENGTH * Math.sin(rotations[0].getRadians() + rotations[1].getRadians())
        );
    }

    public Rotation2d[] solve(Translation2d position, boolean bumper) {
        double newX = position.getX() + SwerveConstants.X_LENGTH_METERS / 2;
        double newY = position.getY() - FourBarConstants.CHASSIS_TO_ARM - FourBarConstants.WHEEL_TO_CHASSIS;
        Translation2d updatedPos = bumper ? new Translation2d(newX, newY) : position;

        double rotElbow;
        double rotShoulder;

        if (updatedPos.getY() > (Units.inchesToMeters(24) - FourBarConstants.CHASSIS_TO_ARM - FourBarConstants.WHEEL_TO_CHASSIS)) {
            rotElbow = -1 * Math.acos((Math.pow(updatedPos.getX(), 2) + Math.pow(updatedPos.getY(), 2) - Math.pow(FourBarConstants.SHOULDER_LENGTH, 2) - Math.pow(FourBarConstants.ELBOW_LENGTH, 2))/(2 * FourBarConstants.SHOULDER_LENGTH * FourBarConstants.ELBOW_LENGTH));
        }
        else {
            rotElbow = 1 * Math.acos((Math.pow(updatedPos.getX(), 2) + Math.pow(updatedPos.getY(), 2) - Math.pow(FourBarConstants.SHOULDER_LENGTH, 2) - Math.pow(FourBarConstants.ELBOW_LENGTH, 2))/(2 * FourBarConstants.SHOULDER_LENGTH * FourBarConstants.ELBOW_LENGTH));
        }
        rotShoulder = Math.atan(updatedPos.getY()/updatedPos.getX()) - Math.atan((FourBarConstants.ELBOW_LENGTH * Math.sin(rotElbow))/(FourBarConstants.SHOULDER_LENGTH + FourBarConstants.ELBOW_LENGTH * Math.cos(rotElbow)));
        if (Double.isNaN(rotElbow) || Double.isNaN(rotShoulder) || rotElbow > FourBarConstants.ELBOW_MAX.getRadians() || rotElbow < FourBarConstants.ELBOW_MIN.getRadians() || rotShoulder > FourBarConstants.SHOULDER_MAX.getRadians() || rotShoulder < FourBarConstants.SHOULDER_MIN.getRadians()) {
            rotElbow *= -1;
            rotShoulder = Math.atan(updatedPos.getY()/updatedPos.getX()) - Math.atan((FourBarConstants.ELBOW_LENGTH * Math.sin(rotElbow))/(FourBarConstants.SHOULDER_LENGTH + FourBarConstants.ELBOW_LENGTH * Math.cos(rotElbow)));

            if (Double.isNaN(rotElbow) || Double.isNaN(rotShoulder) || rotElbow > FourBarConstants.ELBOW_MAX.getRadians() || rotElbow < FourBarConstants.ELBOW_MIN.getRadians() || rotShoulder > FourBarConstants.SHOULDER_MAX.getRadians() || rotShoulder < FourBarConstants.SHOULDER_MIN.getRadians()) {
                return new Rotation2d[] {null, null};
            }
        }

        return new Rotation2d[] {Rotation2d.fromRadians(rotShoulder), Rotation2d.fromRadians(rotElbow)};
    }

//    private Boolean[] checkIllegal(Rotation2d[] rotations) {
//        Rotation2d[] shoulderConstant = new Rotation2d[] {Rotation2d.fromRadians(inputs[0].armPositionRad), rotations[1]};
//        Rotation2d[] elbowConstant = new Rotation2d[] {rotations[0], Rotation2d.fromRadians(inputs[1].armPositionRad)};
////        if (forward()) {
////
////        }
//    }
}