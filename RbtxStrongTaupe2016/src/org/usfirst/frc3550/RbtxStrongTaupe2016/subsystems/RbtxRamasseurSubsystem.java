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
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class RbtxRamasseurSubsystem extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final CANTalon m_moteur = RobotMap.moteurRamasseur;
    private final DigitalInput m_limitSwitch = RobotMap.limitSwitchRamasseur;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    private static final double DIRECTIONASPIRER = 1;
    private static final double DIRECTIONEJECTER = -1;
    private static final double VITESSEASPIRER = 1;
    private static final double VITESSEEJECTER = 1;
    private static final double VITESSEZERO = 0;
    
    private static final boolean UPPERBALL_ISACTIVE = true; // !! verif type
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
    	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    
    public void aspirer() {
    	// AJOUTER QUAND LIMITSWITCH INSTALLE
    	//if (m_limitSwitch.get() == UPPERBALL_ISACTIVE){
    		//stop();
    	//}
    	//else{
    	    m_moteur.set(DIRECTIONASPIRER*VITESSEASPIRER);
    	//}
    }
    	
    
    
    public void ejecter() {
    	m_moteur.set(DIRECTIONEJECTER*VITESSEEJECTER);
    }
    
    public void stop() {
    	m_moteur.set(VITESSEZERO);
    }
}

