package org.usfirst.frc3550.RbtxStrongTaupe2016.commands;

import org.usfirst.frc3550.RbtxStrongTaupe2016.Robot;
import org.usfirst.frc3550.RbtxStrongTaupe2016.subsystems.RbtxDeplacementSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class RbtxDriveFixedDistanceCommand extends Command {
	
	private static final double SONAR_SUPPLYVOLTAGE = 5.0;
	private static final double SONAR_COUNTERRESOLUTION = 512.0;
	private static final double SONAR_VOLTAGE2DISTANCE = SONAR_SUPPLYVOLTAGE / SONAR_COUNTERRESOLUTION;
	double setpoint;
	

    public RbtxDriveFixedDistanceCommand(double setpoint) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.deplacement);
    	this.setpoint = setpoint;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.deplacement.setSetpoint(setpoint);
    	Robot.deplacement.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("position lue: ", Robot.deplacement.getPosition());
    	SmartDashboard.putNumber("setpoint: ", setpoint);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	
    	
    	SmartDashboard.putNumber("difference : ", Math.abs(Robot.deplacement.getPosition()-setpoint));
        return Math.abs(Robot.deplacement.getPosition()-setpoint) < 0.02;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.deplacement.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	Robot.deplacement.disable();
    }
}
