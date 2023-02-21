package com.team3181.frc2023.subsystems.fourbar;


import com.team3181.frc2023.Constants;
import com.team3181.frc2023.Constants.FourBarConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class FourBar extends SubsystemBase {

    private final ArmIO[] armIO;
    private final ArmIOInputsAutoLogged[] armInputs = new ArmIOInputsAutoLogged[]{new ArmIOInputsAutoLogged(), new ArmIOInputsAutoLogged()};

    private final static FourBar INSTANCE = new FourBar(Constants.RobotConstants.SHOULDER, Constants.RobotConstants.ELBOW);

    public static FourBar getInstance() {
        return INSTANCE;
    }

    private FourBar(ArmIO shoulderIO, ArmIO elbowIO) {
        armIO = new ArmIO[]{shoulderIO, elbowIO};

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

    public Rotation2d[] solve(Translation2d position) {
        double rotElbow = - 1 * Math.acos((Math.pow(position.getX(), 2) + Math.pow(position.getY(), 2) - Math.pow(FourBarConstants.SHOULDER_LENGTH, 2) - Math.pow(FourBarConstants.ELBOW_LENGTH, 2))/(2 * FourBarConstants.SHOULDER_LENGTH * FourBarConstants.ELBOW_LENGTH));
        double rotShoulder = Math.atan(position.getY()/position.getX()) + Math.atan((FourBarConstants.ELBOW_LENGTH * Math.sin(rotElbow))/(FourBarConstants.SHOULDER_LENGTH + FourBarConstants.ELBOW_LENGTH * Math.cos(rotElbow)));
        return new Rotation2d[] {Rotation2d.fromRadians(rotShoulder), Rotation2d.fromRadians(rotElbow)};
    }
}