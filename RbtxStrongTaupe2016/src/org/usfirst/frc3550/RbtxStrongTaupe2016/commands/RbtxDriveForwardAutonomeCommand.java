package org.usfirst.frc3550.RbtxStrongTaupe2016.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class RbtxDriveForwardAutonomeCommand extends CommandGroup {
    
    public  RbtxDriveForwardAutonomeCommand(double distance, double Speed) {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	addSequential(new RbtxBrasDescendreAutonomeCommand());
    	addSequential(new RbtxDriveToDistanceWithEncoders(distance, Speed));
    	//30 represents the angle in degree. The robot turns 60 degrees clockwise
    	//addSequential(new RbtxAutoDriveWithEncoderAndGyroCommand(0, 0, turnAngle));
    	//addSequential(new RbtxDriveToDistanceWithEncoders(distance2, Speed));
    	//addSequential(new RbtxShooterAutomatiqueCommand());
  	  	

    }
}
