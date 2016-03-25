package org.usfirst.frc3550.RbtxStrongTaupe2016.commands;


import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc3550.RbtxStrongTaupe2016.Robot;
import org.usfirst.frc3550.RbtxStrongTaupe2016.RobotMap;

/**
 * This command drives the robot over a given distance with simple proportional
 * control This command will drive a given distance limiting to a maximum speed.
 */
public class RbtxDriveToDistanceWithEncoders extends Command {
	
	private PIDController encoderPID;
	private double driveForwardSpeed;
	private double distance;
	private double error;
	private final double TOLERANCE = 0.001;
	private final double KP_encoder = -2; //-2 / 5.0;
	private final double KD_encoder=0;
	private final double KI_encoder=0;

	/*public RbtxDriveToDistanceWithEncoders() {
		this(10, 0.5);
	}

	public RbtxDriveToDistanceWithEncoders(double dist) {
		this(dist, 0.5);
	}*/

    public RbtxDriveToDistanceWithEncoders(double distance, double maxSpeed) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.deplacement);
    	//distance = dist;
    	//driveForwardSpeed = maxSpeed;
    	this.distance = distance;
    	this.driveForwardSpeed = maxSpeed;
   
    	PIDSource  encoderDistanceRight = new PIDSource(){
    		 PIDSourceType m_sourceType = PIDSourceType.kDisplacement;
    		public double pidGet(){
    			return Robot.deplacement.getDistance();
    		}

			@Override
			public void setPIDSourceType(PIDSourceType pidSource) {
				// TODO Auto-generated method stub
				m_sourceType = pidSource;
				
			}

			@Override
			public PIDSourceType getPIDSourceType() {
				// TODO Auto-generated method stub
				return m_sourceType;
			}
    	};
    	
    	PIDOutput targetDistanceWrite = new PIDOutput() {
			
			@Override
			public void pidWrite(double output) {
				// TODO Auto-generated method stub
				driveForwardSpeed = output;
				if(driveForwardSpeed > 0.8) 
					driveForwardSpeed = 0.8;
				if(driveForwardSpeed < -0.8)
					driveForwardSpeed = -0.8;				
			}
		};
		
		encoderPID = new PIDController( KP_encoder,  KI_encoder,  KD_encoder,0, encoderDistanceRight, targetDistanceWrite);
		encoderPID.setAbsoluteTolerance(0.01);
    }
    
	protected void initialize() {
		Robot.deplacement.getRightEncoder().reset();
		encoderPID.reset();
		encoderPID.enable();
		encoderPID.setSetpoint(distance);
		setTimeout(5);
	}

	protected void execute() {
		SmartDashboard.putNumber("Right Encoder Distance set: ", distance);
		error = (distance - Robot.deplacement.getRightEncoder().getDistance());
		SmartDashboard.putNumber("Right Encoder Error: ", error);
	    Robot.deplacement.drive(-1*driveForwardSpeed, 0);
	    SmartDashboard.putNumber("Right Distance PID : ", Robot.deplacement.getDistance());
	}

	protected boolean isFinished() {
		return (Math.abs(error) <= TOLERANCE)|| isTimedOut();
		//return (encoderPID.onTarget() || isTimedOut());
		//return encoderPID.onTarget();
	}

	protected void end() {
		Robot.deplacement.stop();
    	encoderPID.reset();
    	encoderPID.disable();
	}

	protected void interrupted() {
		end();
	}
}
