package com.team3181.frc2023.subsystems.fourbar;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FourBar extends SubsystemBase {

    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.

    private final static FourBar INSTANCE = new FourBar();

    public static FourBar getInstance() {
        return INSTANCE;
    }

    private FourBar() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }
}

