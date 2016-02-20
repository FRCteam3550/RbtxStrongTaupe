// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc3550.RbtxStrongTaupe2016.subsystems;

import org.usfirst.frc3550.RbtxStrongTaupe2016.RobotMap;
import org.usfirst.frc3550.RbtxStrongTaupe2016.commands.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class RbtxBras extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final SpeedController m_bras = RobotMap.rbtxBrasm_bras;
    private final Encoder m_EncodeurBras = RobotMap.rbtxBrasm_EncodeurBras;
    private final DigitalInput m_LSHaut = RobotMap.rbtxBrasm_LSHaut;
    private final DigitalInput m_LSBas = RobotMap.rbtxBrasm_LSBas;
    
    static final double vitesseBrasHaut = 1;
    static final double vitesseBrasBas = -1;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS


    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    
    public void RbtxBrasStop(){
    	m_bras.set(0);
    }
    
    public void RbtxBrasHaut(){
    	m_bras.set(vitesseBrasHaut);
    }
    
    public void RbtxBrasBas(){
    	m_bras.set(vitesseBrasBas);
    }
    
}

