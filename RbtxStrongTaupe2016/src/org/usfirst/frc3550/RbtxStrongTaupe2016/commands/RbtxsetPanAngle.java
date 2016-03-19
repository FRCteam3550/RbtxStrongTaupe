package org.usfirst.frc3550.RbtxStrongTaupe2016.commands;

import org.usfirst.frc3550.RbtxStrongTaupe2016.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RbtxsetPanAngle extends Command {
	
	double angle;

    public RbtxsetPanAngle(double angle) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.bras);
    	this.angle = angle;
    	//Robot.bras.stopCamera();
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.bras.tourneCameraPan(angle);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	//Robot.bras.stopCamera();	
    	
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
