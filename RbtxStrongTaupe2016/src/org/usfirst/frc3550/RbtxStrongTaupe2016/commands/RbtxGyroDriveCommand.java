package org.usfirst.frc3550.RbtxStrongTaupe2016.commands;

import org.usfirst.frc3550.RbtxStrongTaupe2016.Robot;
import org.usfirst.frc3550.RbtxStrongTaupe2016.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @param <voltsPerDegreePerSecond>
 *
 */
public class RbtxGyroDriveCommand<voltsPerDegreePerSecond> extends Command {
	
    public RbtxGyroDriveCommand() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.deplacement);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	RobotMap.gyro.reset();
    	//gyro calibration constant, may need to be adjusted; 
        //gyro value of 360 is set to correspond to one full revolution
        // double voltsPerDegreePerSecond = .0090; 
         double voltsPerDegreePerSecond = .0125;

    	RobotMap.gyro.setSensitivity(voltsPerDegreePerSecond);
    	Robot.deplacement.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("gyro position Commande", Robot.deplacement.getPosition());
    	//Robot.deplacement.gyroDrive(Robot.deplacement.getPosition());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    	
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.deplacement.disable();
    	RobotMap.gyro.reset();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
