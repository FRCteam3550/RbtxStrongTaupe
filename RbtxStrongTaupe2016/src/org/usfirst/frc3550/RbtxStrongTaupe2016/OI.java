// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc3550.RbtxStrongTaupe2016;

import org.usfirst.frc3550.RbtxStrongTaupe2016.commands.*;
import org.usfirst.frc3550.RbtxStrongTaupe2016.subsystems.RbtxDeplacementSubsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.buttons.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released  and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());


	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	private Joystick gamePadPilote;
	private Joystick joystickCoPilote;

	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	public OI() {
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

		// Pilote
		gamePadPilote = new Joystick(0);
		int BUTTON_X = 1;
		int BUTTON_A = 2;
		int BUTTON_B = 3;
		int BUTTON_Y = 4;
		int BUTTON_LB = 5;
		
		//Button buttonInverseDrive = new JoystickButton(gamePadPilote, 4);
		//buttonInverseDrive.whenPressed(new RbtxArcadeDriveInversedCommand());
		
		//Button buttonDrive = new JoystickButton(gamePadPilote, 2);
		//buttonDrive.whenPressed(new RbtxArcadeDriveCommand());
		
		//Button buttonSlowDrive = new JoystickButton(gamePadPilote, 3);
		//buttonSlowDrive.whenPressed(new RbtxSlowDriveCommand());
		
		//Button buttonInverseSlowDrive = new JoystickButton(gamePadPilote, 1);
		//buttonInverseSlowDrive.whenPressed(new RbtxSlowDriveInverseCommand());
		
		Button buttonInverseDrive = new JoystickButton(gamePadPilote, BUTTON_Y);
		buttonInverseDrive.whenPressed(new RbtxInverseTankDriveCommand());
		
		Button buttonDrive = new JoystickButton(gamePadPilote, BUTTON_A);
		buttonDrive.whenPressed(new RbtxTankDriveCommand());
		
		Button buttonInverseSlowTankDrive = new JoystickButton(gamePadPilote, BUTTON_X);
		buttonInverseSlowTankDrive.whenPressed(new RbtxInverseSlowTankDriveCommand());
				
		Button buttonSlowTankDrive = new JoystickButton(gamePadPilote, BUTTON_B);
		buttonSlowTankDrive.whenPressed(new RbtxSlowTankDriveCommand());
		
		Button buttonAvanceAvecEncodeurs = new JoystickButton(gamePadPilote,BUTTON_LB); // to be tested once the encoders are installed on the robot
		//buttonAvanceAvecEncodeurs.whenPressed(new RbtxDriveToDistanceWithEncoders(2.0, 0.7)); // a 3.36 ft de la tour
		                                                                    
		Button buttonVaDevantTourTest = new JoystickButton(gamePadPilote, 8);
		//buttonVaDevantTourTest.whenPressed(new RbtxDriveToTargetWithSonarCommand(RbtxDeplacementSubsystem.TOUR));
		
		Button buttonGyroTest = new JoystickButton(gamePadPilote, 10);
		buttonGyroTest.whenPressed(new RbtxGyroDriveCommand());
		
		Button buttonGyroTurn = new JoystickButton(gamePadPilote, 9);
		buttonGyroTurn.whenActive(new RbtxAutoDriveWithEncoderAndGyroCommand(0, 0, 45));
		
		
		// Copilote
		joystickCoPilote = new Joystick(1);
		final int BOUTON_1   = 1;
	    final int BOUTON_3   = 3;
	    final int BOUTON_5   = 5;
	    final int BOUTON_7   = 7;
	    final int BOUTON_8   = 8;
	    final int BOUTON_9   = 9;
	    final int BOUTON_4   = 4;
	    final int BOUTON_6   = 6;
	    final int BOUTON_10  = 10;  
		final int BOUTON_11  = 11;
	    final int BOUTON_12  = 12;
		
		//camera selection
		Button buttonLancerAutomatique = new JoystickButton(joystickCoPilote, BOUTON_11);
		buttonLancerAutomatique.whenPressed(new RbtxShooterAutomatiqueCommand()); 
		//Button buttonCameraSelection = new JoystickButton(joystickCoPilote, BOUTON_11);
		
		Button buttonCameraTourne = new JoystickButton(joystickCoPilote, BOUTON_10);
		//buttonCameraTourne.whileHeld(new RbtxsetTiltAngle(90)); //Angle positif pour droite
		
		Button buttonCameraRepos = new JoystickButton(joystickCoPilote, BOUTON_12);
		//buttonCameraRepos.whenActive(new RbtxsetTiltAngle(0)); //Angle positif pour droite

		// Boutons bras
		Button buttonBrasMonter = new JoystickButton(joystickCoPilote, BOUTON_5);
		Button buttonBrasDescendre = new JoystickButton(joystickCoPilote, BOUTON_3);

		buttonBrasMonter.whenPressed(new RbtxBrasMonterCommand());
		buttonBrasDescendre.whenPressed(new RbtxBrasDescendreCommand());

		// Boutons ramasseur
		Button buttonRamasseurAspirer = new JoystickButton(joystickCoPilote, BOUTON_9);
		Button buttonRamasseurEjecter = new JoystickButton(joystickCoPilote, BOUTON_4);

		buttonRamasseurAspirer.whenPressed(new RbtxRamasseurAspirerCommand());
		buttonRamasseurEjecter.whileHeld(new RbtxRamasseurEjecterCommand());

		// Boutons accelerateur
		Button buttonAccelerateur = new JoystickButton(joystickCoPilote, BOUTON_6);
		//Button buttonLanceurActifmin = new JoystickButton(joystickCoPilote, 7);

		buttonAccelerateur.whenPressed(new RbtxAccelerPourLancerCommand());
		//buttonLanceurActifmin.whileHeld(new RbtxLanceurCommand());
		
		Button buttonAccelerateurSetSpeedPID = new JoystickButton(joystickCoPilote, BOUTON_8);
		//buttonAccelerateurSetSpeedPID.whenPressed(new RbtxSetShooterMaxSpeedPIDCommand(0.95));// to be tested ASAP
		
		// Bouton pour lancer
		Button buttonLancer = new JoystickButton(joystickCoPilote, BOUTON_1);
		buttonLancer.whenPressed(new RbtxLancerCommand());
		//buttonLancer.whenPressed(new RbtxAutoForwardCommand());
		Button buttonArretLancer = new JoystickButton(joystickCoPilote, BOUTON_7);
		buttonArretLancer.whenPressed(new RbtxArreterAccelerateurCommand());

		// SmartDashboard Buttons
		SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
		SmartDashboard.putData("ArcadeDrive", new RbtxArcadeDriveCommand());
		SmartDashboard.putData("BrasDescendre", new RbtxBrasDescendreCommand());
		SmartDashboard.putData("BrasMonter", new RbtxBrasMonterCommand());
		SmartDashboard.putData("RamasseurAspirer", new RbtxRamasseurAspirerCommand()); 
		SmartDashboard.putData("RamasseurEjecter", new RbtxRamasseurEjecterCommand());
		//SmartDashboard.putData("Accelerateur", new RbtxAccelererCommand());
		
		//SmartDashboard.putBoolean("BoutonAccelerateur", buttonAccelerateur.get());

		// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
	}

    public Joystick getGamePadPilote() { //gamePadPilote on usb0
		return gamePadPilote;
	}
    
	public Joystick getJoystickCoPilote() { //CoPilote joystick on usb1
		return joystickCoPilote;
	}
	
	public double getgamePadPiloteX() { //x axis on the gamePadPilote right joystick
		return gamePadPilote.getRawAxis(2);
	}
	
	public double getgamePadPiloteY() { //y axis on the gamePadPilote left joystick
		return gamePadPilote.getRawAxis(1);
	}
	
	public double getgamePadPiloteYLeft() { //Malcolm's choice for gamePadPilote tank drive mode
		return gamePadPilote.getRawAxis(1);
	}
	
	public double getgamePadPiloteYRight() {//Malcolm's choice for gamePadPilote tank drive mode
		return gamePadPilote.getRawAxis(3);
			
	}
	
	public double getgamePadaxis4() {//Malcolm's choice for gamePadPilote tank drive mode
		return gamePadPilote.getRawAxis(4);
			
	}
	
	public double getgamePadaxis5() {//Malcolm's choice for gamePadPilote tank drive mode
		return gamePadPilote.getRawAxis(5);
			
	}
	public void log(){
		SmartDashboard.putBoolean("X-SlowInverseTankDrive", Robot.oi.getGamePadPilote().getRawButton(1));
		SmartDashboard.putBoolean("A-TankDrive", Robot.oi.getGamePadPilote().getRawButton(2));
		SmartDashboard.putBoolean("B-SlowTankDrive", Robot.oi.getGamePadPilote().getRawButton(3));
		SmartDashboard.putBoolean("Y-TankDriveInverse", Robot.oi.getGamePadPilote().getRawButton(4));
		SmartDashboard.putBoolean("LB-DriveWithEncoder", Robot.oi.getGamePadPilote().getRawButton(5));
		SmartDashboard.putBoolean("bouton6", Robot.oi.getGamePadPilote().getRawButton(6));
		SmartDashboard.putBoolean("bouton11", Robot.oi.getGamePadPilote().getRawButton(11));
		SmartDashboard.putBoolean("RT-SonarTargetPID", Robot.oi.getGamePadPilote().getRawButton(8));
		SmartDashboard.putBoolean("BACK-Turn-test", Robot.oi.getGamePadPilote().getRawButton(9));
		SmartDashboard.putBoolean("START-Gyro-Assist", Robot.oi.getGamePadPilote().getRawButton(10));
		SmartDashboard.getNumber("axe4",getgamePadaxis4() );
		SmartDashboard.getNumber("axe5",getgamePadaxis5() );
	}
	
	public void logCoPiloteJoystick(){
		SmartDashboard.putNumber("Axis 1", Robot.oi.joystickCoPilote.getRawAxis(1));
		SmartDashboard.putNumber("Axis 2", Robot.oi.joystickCoPilote.getRawAxis(2));
		SmartDashboard.putNumber("Axis 3", Robot.oi.joystickCoPilote.getRawAxis(3));
		//SmartDashboard.putNumber("Axis 4", Robot.oi.joystickCoPilote.getRawAxis(4));
	}
}

