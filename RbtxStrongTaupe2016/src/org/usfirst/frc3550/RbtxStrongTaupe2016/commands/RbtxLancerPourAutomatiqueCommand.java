package org.usfirst.frc3550.RbtxStrongTaupe2016.commands;

import org.usfirst.frc3550.RbtxStrongTaupe2016.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RbtxLancerPourAutomatiqueCommand extends Command {

    public RbtxLancerPourAutomatiqueCommand() {
        // Use requires() here to declare subsystem dependencies
         requires(Robot.ramasseur);
         //requires(Robot.accelerateur);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.ramasseur.stop(); 
    	Timer.delay(3);
    	setTimeout(8);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Robot.ramasseur.stop(); 
    	//Timer.delay(3);
    	Robot.ramasseur.aspirerLeBallon();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	return isTimedOut();
  
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.ramasseur.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
