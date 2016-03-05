package org.usfirst.frc3550.RbtxStrongTaupe2016.subsystems;

import org.usfirst.frc3550.RbtxStrongTaupe2016.RobotMap;

import edu.wpi.first.wpilibj.AnalogGyro;
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
 The DriveTrain subsystem incorporates the sensors and actuators attached to
 * the robots chassis. These include four drive motors, a left and right encoder
 * and a gyro. It reads in information about it's speed and position.
 *
 */
public class RbtxDeplacementSubsystem extends PIDSubsystem {

	private static final double Kp_gyro = 1.98; // 2.35 3
	private static final double Ki_gyro = 0.00583; // 0.01
	private static final double Kd_gyro = 0.00008;

	public static final double TOUR = 0.25;
	public static final double TOUR2 = 0.8;
	public static final double SLOWSPEED = 0.75;
	public static final boolean SENSITIVITY = true;
	
	private double currentHeading;
	
	// Subsystem devices
	private RobotDrive m_drive = RobotMap.drive;
	private AnalogInput m_rangefinder = RobotMap.forwardSonar;
	private AnalogGyro m_gyro  = RobotMap.gyro;

	// Initialize your subsystem here
	public RbtxDeplacementSubsystem() {
		super("RbtxDeplacementSubsystem", Kp_gyro, Ki_gyro, Kd_gyro);

		setAbsoluteTolerance(0.2);
		getPIDController().setContinuous(false);
		LiveWindow.addActuator("RbtxDeplacementSubsystem", "GyPID Controller", getPIDController());
		// Use these to get going:
		// setSetpoint() - Sets where the PID controller should move the system
		// to
		// enable(); //Enables the PID controller.
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new RbtxArcadeDriveCommand());
		//setDefaultCommand(new RbtxGyroDriveCommand());
		setDefaultCommand(new RbtxTankDriveCommand());
	}

	/**
	 * The log method puts interesting information to the SmartDashboard.
	 */
	public void log() {
		SmartDashboard.putNumber("GyroPIDinput", returnPIDInput());
	}

	protected double returnPIDInput() {
		// Return your input value for the PID loop
		// e.g. a sensor, like a potentiometer:
		// yourPot.getAverageVoltage() / kYourMaxVoltage;
		currentHeading = m_gyro.getAngle();
		
		return m_gyro.getAngle();
	}

	protected void usePIDOutput(double output) {
		// Use output to drive your system, like a motor
		// e.g. yourMotor.set(output);
		//SmartDashboard.putNumber("Current PIDOutput", output);
		
		//double speed = output;
		//if (speed > 0.8)
			//speed = 0.8;
		//if (currentHeading <= TOUR+0.04)
			//speed = 0.5;
		
	 gyroDrive(output);
	}
	
	public double getDistanceToTarget(){
		   return m_rangefinder.getVoltage();	
	}

	public void driveStraight(double speed) {
		// Configure the RobotDrive to reflect the fact that all our motors are
	    // wired backwards and our drivers sensitivity preferences.
		RobotMap.moteurDeplacementAvantGauche.setInverted(true);
		RobotMap.moteurDeplacementArriereGauche.setInverted(true);
		RobotMap.moteurDeplacementAvantDroite.setInverted(true);
		RobotMap.moteurDeplacementArriereDroite.setInverted(true);
		m_drive.drive(speed, 0);
	}
	
	
	public void gyroDrive(double rotateValue) {
		double moveValue = Robot.oi.getgamePadPiloteY();
		//double rotateValue = Robot.oi.getgamePadPiloteX();
		
		// Configure the RobotDrive to reflect the fact that all our motors are
		// wired backwards and our drivers sensitivity preferences.
		RobotMap.moteurDeplacementAvantGauche.setInverted(true);
		RobotMap.moteurDeplacementArriereGauche.setInverted(true);
		RobotMap.moteurDeplacementAvantDroite.setInverted(true);
		RobotMap.moteurDeplacementArriereDroite.setInverted(true);
		m_drive.arcadeDrive(moveValue, rotateValue);
	}
	

	public void stop() {
		m_drive.drive(0, 0);
	}

	public void drive(double moveValue, double rotateValue) {
		//double moveValue = Robot.oi.getgamePadPiloteY();
		//double rotateValue = Robot.oi.getgamePadPiloteX();
		
		// Configure the RobotDrive to reflect the fact that all our motors are
		// wired backwards and our drivers sensitivity preferences.
		RobotMap.moteurDeplacementAvantGauche.setInverted(true);
		RobotMap.moteurDeplacementArriereGauche.setInverted(true);
		RobotMap.moteurDeplacementAvantDroite.setInverted(true);
		RobotMap.moteurDeplacementArriereDroite.setInverted(true);
		m_drive.arcadeDrive(moveValue, rotateValue, SENSITIVITY);
	}

	public void inverseDrive(double moveValue, double rotateValue) {
		//double moveValue = Robot.oi.getgamePadPiloteY();
		//double rotateValue = Robot.oi.getgamePadPiloteX();
		
		// Configure the RobotDrive to reflect the fact that all our motors are
		// wired backwards and our drivers sensitivity preferences.
		RobotMap.moteurDeplacementAvantGauche.setInverted(false);
		RobotMap.moteurDeplacementArriereGauche.setInverted(false);
		RobotMap.moteurDeplacementAvantDroite.setInverted(false);
		RobotMap.moteurDeplacementArriereDroite.setInverted(false);
		m_drive.arcadeDrive(moveValue, -1 * rotateValue, SENSITIVITY);
	}
	
	public void driveSlow(double moveValue, double rotateValue) {
		//double moveValue = Robot.oi.getgamePadPiloteY();
		//double rotateValue = Robot.oi.getgamePadPiloteX();
		
		// Configure the RobotDrive to reflect the fact that all our motors are
		// wired backwards and our drivers sensitivity preferences.
		RobotMap.moteurDeplacementAvantGauche.setInverted(true);
		RobotMap.moteurDeplacementArriereGauche.setInverted(true);
		RobotMap.moteurDeplacementAvantDroite.setInverted(true);
		RobotMap.moteurDeplacementArriereDroite.setInverted(true);
		m_drive.arcadeDrive(SLOWSPEED*moveValue, SLOWSPEED*rotateValue, SENSITIVITY);
	}
	
	public void inverseDriveSlow(double moveValue, double rotateValue) {
		//double moveValue = Robot.oi.getgamePadPiloteY();
		//double rotateValue = Robot.oi.getgamePadPiloteX();
		
		// Configure the RobotDrive to reflect the fact that all our motors are
		// wired backwards and our drivers sensitivity preferences.
		RobotMap.moteurDeplacementAvantGauche.setInverted(false);
		RobotMap.moteurDeplacementArriereGauche.setInverted(false);
		RobotMap.moteurDeplacementAvantDroite.setInverted(false);
		RobotMap.moteurDeplacementArriereDroite.setInverted(false);
		m_drive.arcadeDrive(SLOWSPEED*moveValue, -1*SLOWSPEED*rotateValue, SENSITIVITY);
	}
	
	public void driveTank(double leftStick, double rightStick) {
		//leftStick = Robot.oi.getgamePadPiloteYLeft();
		//rightStick = Robot.oi.getgamePadPiloteYRight();
		
		// Configure the RobotDrive to reflect the fact that all our motors are
		// wired backwards and our drivers sensitivity preferences.
		RobotMap.moteurDeplacementAvantGauche.setInverted(true);
		RobotMap.moteurDeplacementArriereGauche.setInverted(true);
		RobotMap.moteurDeplacementAvantDroite.setInverted(true);
		RobotMap.moteurDeplacementArriereDroite.setInverted(true);
		//m_drive.arcadeDrive(SLOWSPEED*moveValue, SLOWSPEED*rotateValue, SENSITIVITY);
		m_drive.tankDrive(leftStick, rightStick, SENSITIVITY);
	}
	
	public void inverseTankDrive(double leftStick, double rightStick) {
		//leftStick = Robot.oi.getgamePadPiloteYLeft();
		//rightStick = Robot.oi.getgamePadPiloteYRight();
		
		// Configure the RobotDrive to reflect the fact that all our motors are
		// wired backwards and our drivers sensitivity preferences.
		RobotMap.moteurDeplacementAvantGauche.setInverted(false);
		RobotMap.moteurDeplacementArriereGauche.setInverted(false);
		RobotMap.moteurDeplacementAvantDroite.setInverted(false);
		RobotMap.moteurDeplacementArriereDroite.setInverted(false);
		m_drive.tankDrive(rightStick, leftStick, SENSITIVITY);
	}
	
	public void SlowTankDrive(double leftStick, double rightStick) {
		//leftStick = Robot.oi.getgamePadPiloteYLeft();
		//rightStick = Robot.oi.getgamePadPiloteYRight();
		
		// Configure the RobotDrive to reflect the fact that all our motors are
		// wired backwards and our drivers sensitivity preferences.
		RobotMap.moteurDeplacementAvantGauche.setInverted(true);
		RobotMap.moteurDeplacementArriereGauche.setInverted(true);
		RobotMap.moteurDeplacementAvantDroite.setInverted(true);
		RobotMap.moteurDeplacementArriereDroite.setInverted(true);
		//m_drive.arcadeDrive(SLOWSPEED*moveValue, SLOWSPEED*rotateValue, SENSITIVITY);
		m_drive.tankDrive(leftStick*SLOWSPEED, rightStick*SLOWSPEED, SENSITIVITY);
	}
	
	public void inverseSlowTankDrive(double leftStick, double rightStick) {
		//leftStick = Robot.oi.getgamePadPiloteYLeft();
		//rightStick = Robot.oi.getgamePadPiloteYRight();
		
		// Configure the RobotDrive to reflect the fact that all our motors are
		// wired backwards and our drivers sensitivity preferences.
		RobotMap.moteurDeplacementAvantGauche.setInverted(false);
		RobotMap.moteurDeplacementArriereGauche.setInverted(false);
		RobotMap.moteurDeplacementAvantDroite.setInverted(false);
		RobotMap.moteurDeplacementArriereDroite.setInverted(false);
		m_drive.tankDrive(rightStick*SLOWSPEED, leftStick*SLOWSPEED, SENSITIVITY);
	}
}
