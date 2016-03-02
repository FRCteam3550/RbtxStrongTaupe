package org.usfirst.frc3550.RbtxStrongTaupe2016.subsystems;

import org.usfirst.frc3550.RbtxStrongTaupe2016.RobotMap;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc3550.RbtxStrongTaupe2016.Robot;
import org.usfirst.frc3550.RbtxStrongTaupe2016.commands.*;
import edu.wpi.first.wpilibj.Joystick;

/**
 *
 */
public class RbtxDeplacementSubsystem extends PIDSubsystem {

	private static final double Kp = 1.98; // 2.35 3
	private static final double Ki = 0.00583; // 0.01
	private static final double Kd = 0.00008;

	public static final double TOUR = 0.3;
	public static final double TOUR2 = 0.8;

	private RobotDrive m_drive = RobotMap.drive;
	private AnalogInput m_rangefinder = RobotMap.forwardSonar;

	// Initialize your subsystem here
	public RbtxDeplacementSubsystem() {
		super("RbtxDeplacementSubsystem", Kp, Ki, Kd);

		setAbsoluteTolerance(0.2);
		getPIDController().setContinuous(false);
		LiveWindow.addActuator("RbtxDeplacementSubsystem", "PIDSubsystem Controller", getPIDController());
		// Use these to get going:
		// setSetpoint() - Sets where the PID controller should move the system
		// to
		// enable(); //Enables the PID controller.
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new RbtxArcadeDriveCommand());
	}

	/**
	 * The log method puts interesting information to the SmartDashboard.
	 */
	public void log() {
		SmartDashboard.putNumber("PIDInput", returnPIDInput());
	}

	protected double returnPIDInput() {
		// Return your input value for the PID loop
		// e.g. a sensor, like a potentiometer:
		// yourPot.getAverageVoltage() / kYourMaxVoltage;
		return m_rangefinder.getVoltage();
	}

	protected void usePIDOutput(double output) {
		// Use output to drive your system, like a motor
		// e.g. yourMotor.set(output);
		SmartDashboard.putNumber("Current PIDOutput", output);
		driveStraight(output);
	}

	public void driveStraight(double speed) {
		RobotMap.moteurDeplacementAvantGauche.setInverted(true);
		RobotMap.moteurDeplacementArriereGauche.setInverted(true);
		RobotMap.moteurDeplacementAvantDroite.setInverted(true);
		RobotMap.moteurDeplacementArriereDroite.setInverted(true);
		
		if (speed > 0.4) {
			speed = 0.4;
		}
			
		m_drive.drive(speed, 0);
	}

	public void stop() {
		m_drive.drive(0, 0);
	}

	public void drive(Joystick stick) {
		double moveValue = Robot.oi.getgamePadPiloteY();
		double rotateValue = Robot.oi.getgamePadPiloteX();
		RobotMap.moteurDeplacementAvantGauche.setInverted(true);
		RobotMap.moteurDeplacementArriereGauche.setInverted(true);
		RobotMap.moteurDeplacementAvantDroite.setInverted(true);
		RobotMap.moteurDeplacementArriereDroite.setInverted(true);
		m_drive.arcadeDrive(moveValue, rotateValue);
	}

	public void inverseDrive(Joystick stick) {
		double moveValue = Robot.oi.getgamePadPiloteY();
		double rotateValue = Robot.oi.getgamePadPiloteX();
		RobotMap.moteurDeplacementAvantGauche.setInverted(false);
		RobotMap.moteurDeplacementArriereGauche.setInverted(false);
		RobotMap.moteurDeplacementAvantDroite.setInverted(false);
		RobotMap.moteurDeplacementArriereDroite.setInverted(false);
		m_drive.arcadeDrive(moveValue, -1 * rotateValue);
	}

}
