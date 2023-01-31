package com.team3181.frc2023.subsystems.endeffector;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public  class EndEffector extends SubsystemBase
{
        
    // With eager singleton initialization, any static variables/fields used in the 
    // constructor must appear before the "INSTANCE" variable so that they are initialized 
    // before the constructor is called when the "INSTANCE" variable initializes.
    
private final static EndEffector INSTANCE = new EndEffector();
    
public static EndEffector getInstance()
{    
    return INSTANCE;
}

    private EndEffector()
    {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
                //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
            }
}

