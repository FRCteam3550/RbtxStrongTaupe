package org.usfirst.frc3550.RbtxStrongTaupe2016.commands;

import org.usfirst.frc3550.RbtxStrongTaupe2016.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RbtxLancerCommand extends Command {

    public RbtxLancerCommand() {
        // Use requires() here to declare subsystem dependencies
         requires(Robot.ramasseur);
         //requires(Robot.accelerateur);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	setTimeout(6);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.ramasseur.aspirerLeBallon();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
       return isTimedOut();
  
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.ramasseur.stop();
    	//Robot.accelerateur.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
