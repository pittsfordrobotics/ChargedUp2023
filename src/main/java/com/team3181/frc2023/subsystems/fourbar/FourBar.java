package com.team3181.frc2023.subsystems.fourbar;


import com.team3181.frc2023.Constants;
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

    public static void setArmVoltage(ArmIO io, double voltage) {
        io.setVoltage(voltage);
    }

    /** Converts joint angles to the end effector position. */
    public Translation2d forward() {
        return new Translation2d(
                Constants.Dimensions.shoulderJointPositionX
                        + Constants.Dimensions.arm1 * Math.cos(armInputs[0].armPositionRad)
                        + Constants.Dimensions.arm2 * Math.cos(armInputs[0].armPositionRad
                        + armInputs[1].armPositionRad),
                Constants.Dimensions.shoulderJointPositionY
                        + Constants.Dimensions.arm1 * Math.sin(armInputs[0].armPositionRad)
                        + Constants.Dimensions.arm2 * Math.sin(armInputs[0].armPositionRad
                        + armInputs[1].armPositionRad)
        );
    }
}