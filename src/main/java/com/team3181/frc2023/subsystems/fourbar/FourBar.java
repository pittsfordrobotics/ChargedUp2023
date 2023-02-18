package com.team3181.frc2023.subsystems.fourbar;


import com.team3181.frc2023.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FourBar extends SubsystemBase {

    public final ArmIO[] armIO;
    public final ArmIOInputsAutoLogged[] armInputs = new ArmIOInputsAutoLogged[]{new ArmIOInputsAutoLogged(), new ArmIOInputsAutoLogged()};
    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

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

    public static void setArmVoltage(ArmIO io, double voltage) {
        io.setVoltage(voltage);
    }
}

