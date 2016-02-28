package org.usfirst.frc3550.RbtxStrongTaupe2016.subsystems;

import org.usfirst.frc3550.RbtxStrongTaupe2016.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import org.usfirst.frc3550.RbtxStrongTaupe2016.OI;
import org.usfirst.frc3550.RbtxStrongTaupe2016.Robot;
import org.usfirst.frc3550.RbtxStrongTaupe2016.RobotMap;
import org.usfirst.frc3550.RbtxStrongTaupe2016.commands.*;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;

/**
 *
 */
public class RbtxDeplacementSubsystem extends PIDSubsystem {
	
	
	
	
    private static final double Kp = 0.002;
    private static final double Ki = 0.0;
    private static final double Kd = 0.0;
    
    private RobotDrive m_drive = RobotMap.drive;
    private AnalogInput m_rangefinder = RobotMap.forwardSonar;
    
    // Initialize your subsystem here
    public RbtxDeplacementSubsystem() {
    	super("RbtxDeplacementSubsystem", Kp, Ki, Kd);
    	
    	setAbsoluteTolerance(0.2);
        getPIDController().setContinuous(false);
        LiveWindow.addActuator("RbtxDeplacementSubsystem", "PIDSubsystem Controller", getPIDController());
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
       // enable(); //Enables the PID controller.
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
         setDefaultCommand(new RbtxArcadeDriveCommand());
    }
    
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
    	return m_rangefinder.getAverageVoltage();
    }
    
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	   tankDrive(output, output);
    }
    
    public void tankDrive(double left, double right){
    	RobotMap.moteurDeplacementAvantGauche.setInverted(false);
    	RobotMap.moteurDeplacementArriereGauche.setInverted(false);
    	RobotMap.moteurDeplacementAvantDroite.setInverted(false);
    	RobotMap.moteurDeplacementArriereDroite.setInverted(false);
    	m_drive.tankDrive(left, right);
    }
    
    public void stop() {
    	m_drive.drive(0, 0);
    }
    
    public void drive(Joystick stick) {
    	RobotMap.moteurDeplacementAvantGauche.setInverted(true);
    	RobotMap.moteurDeplacementArriereGauche.setInverted(true);
    	RobotMap.moteurDeplacementAvantDroite.setInverted(true);
    	RobotMap.moteurDeplacementArriereDroite.setInverted(true);
    	m_drive.arcadeDrive(stick);
    }
    
    public void inverseDrive(Joystick stick) {
    	double moveValue = Robot.oi.getgamePadPiloteY();
		double rotateValue = Robot.oi.getgamePadPiloteX();
	    RobotMap.moteurDeplacementAvantGauche.setInverted(false);
	    RobotMap.moteurDeplacementArriereGauche.setInverted(false);
	    RobotMap.moteurDeplacementAvantDroite.setInverted(false);
	    RobotMap.moteurDeplacementArriereDroite.setInverted(false);
	    m_drive.arcadeDrive(moveValue, -1* rotateValue);
    }
    
}
