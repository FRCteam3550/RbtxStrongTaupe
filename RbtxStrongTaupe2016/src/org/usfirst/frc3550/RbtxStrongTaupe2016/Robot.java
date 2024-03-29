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

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import org.usfirst.frc3550.RbtxStrongTaupe2016.commands.*;
import org.usfirst.frc3550.RbtxStrongTaupe2016.subsystems.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

import com.ni.vision.NIVision.Image;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	
	private static final double SONAR_SUPPLYVOLTAGE = 5.0;
	private static final double SONAR_COUNTERRESOLUTION = 512.0;
	private static final double SONAR_VOLTAGE2DISTANCE = SONAR_SUPPLYVOLTAGE / SONAR_COUNTERRESOLUTION;
	
	Command autonomousCommand;

	public static OI oi;
	// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
	public static RbtxDeplacementSubsystem deplacement;
	public static RbtxBrasSubsystem bras;
	public static RbtxRamasseurSubsystem ramasseur;
	public static RbtxAccelerateurSubsystem accelerateur;
	
	SendableChooser autoSelecteur;
	
    CameraServer  usbCamera;
	
    // A remettre a NorthBay
     Image frame;
     AxisCamera axisCamera;
	

	// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		RobotMap.init();
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
		deplacement = new RbtxDeplacementSubsystem();
		bras = new RbtxBrasSubsystem();
		ramasseur = new RbtxRamasseurSubsystem();
		accelerateur = new RbtxAccelerateurSubsystem();
		
		// Show what command your subsystem is running on the SmartDashboard
        //SmartDashboard.putData(Robot.accelerateur);
       // SmartDashboard.putData(Robot.bras);
        SmartDashboard.putData(Robot.deplacement);
       // SmartDashboard.putData(Robot.ramasseur);

		// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
		// OI must be constructed after subsystems. If the OI creates Commands
		//(which it very likely will), subsystems are not guaranteed to be
		// constructed yet. Thus, their requires() statements may grab null
		// pointers. Bad news. Don't move it.
		oi = new OI();

		// instantiate the command used for the autonomous period
		// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
		autoSelecteur = new SendableChooser();
        //		
    	autoSelecteur.addDefault("->Robotronix Ne fais rien", new RbtxNeFaitRienCommand());
        autoSelecteur.addObject("->En avant", new RbtxAutoForwardCommand());
        autoSelecteur.addObject("->Reculer", new RbtxReverseAutoCommand());
        autoSelecteur.addObject("->Composer", new RbtxAutoCompoundCommand());
        //DriveEncoder : @inputs distance to drive in ft and speed
        //autoSelecteur.addObject("->Going Forward", new RbtxDriveToDistanceWithEncoders(-13.0, 0.6));
        autoSelecteur.addObject("->Going Forward", new RbtxDriveForwardAutonomeCommand(-13.0, 0.6));
        autoSelecteur.addObject("->Shooter Autonome", new RbtxShooterAutomatiqueCommand());
        //MoveShoot : @input distance to drive in ft
        autoSelecteur.addObject("->MoveShoot:", new RbtxDeplacerStraightEtShooterAutonomeCommand(5));
        //ArmLowMoveTurnMoveShoot : @inputs distanc1 19.09 in ft distance2 8.17 in ft speed 0.5 turnAngle 30 in degrees
        autoSelecteur.addObject("->ArmLowMoveTurnMoveShoot:", new RbtxDeplacerStraightTurnStraightShooterAutonomeCommand(-19.09, -8.20, 0.5, 30));
        //ArmLowMoveShootLow      : @inputs distance1  to drive forward in ft distance2 to drive backward in ft SPEED turnAngle in degrees
        autoSelecteur.addObject("->ArmLowMoveShootLow:", new RbtxDeplacerStraightShooterLowTargetmoveBackAutonomeCommand(-19.09, 8.17, 0.5,30));
        //autoSelecteur.addObject("", new Command()); // Template
        SmartDashboard.putData("Selection Autonomes", autoSelecteur);
        
        axisCamera = new AxisCamera("axis-camera.local"); //A remettre a NorthBay
        usbCamera = CameraServer.getInstance();
        usbCamera.setQuality(50); //be carefull when changing this value, 30 doesnt work
        //the camera name (ex "cam0") can be found through the roborio web interface
        usbCamera.startAutomaticCapture("cam0");
        //axisCamera.getImage(frame);
        // CameraServer.getInstance().setImage(frame);\
        
        RobotMap.gyro.reset();
	}

	/**
	 * This function is called when the disabled button is hit.
	 * You can use it to reset subsystems before shutting down.
	 */
	public void disabledInit() {

	}

	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	public void autonomousInit() {
		// schedule the autonomous command (example)
		autonomousCommand = (Command) autoSelecteur.getSelected();
		if (autonomousCommand != null) autonomousCommand.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	public void teleopInit() {
		RobotMap.encodeurDeplacementDroit.reset();
		RobotMap.encodeurDeplacementGauche.reset();
		RobotMap.encodeurBras.reset();
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) autonomousCommand.cancel();
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		//log(); //Template a developper voir plus bas
		// cameraToggle();
		//Robot.oi.log();
		Robot.oi.logCoPiloteJoystick();
		//RobotMap.moteurAccelerateur.set(0.6);
		
	   /* SmartDashboard.putNumber("gamePadPiloteXaxis: ", Robot.oi.getgamePadPiloteX());
		SmartDashboard.putNumber("gamePadPiloteYaxis: ", Robot.oi.getgamePadPiloteY());
		SmartDashboard.getNumber("axe4",Robot.oi.getgamePadaxis4() );
		SmartDashboard.getNumber("axe5",Robot.oi.getgamePadaxis5()); */
		
		//SmartDashboard.putNumber("JoystickCoPiloteYaxis: ", Robot.oi.joystickCoPilote.getY());
		
		//SmartDashboard.putBoolean("BoutonAccelerateur: ", Robot.oi.joystickCoPilote.getRawButton(6));
		SmartDashboard.putBoolean("BrasEnBas: ", (RobotMap.brasEnBas.get()));
		SmartDashboard.putBoolean("BrasEnHaut: ", !RobotMap.brasEnHaut.get());
		SmartDashboard.putBoolean("BallonPresent: ", RobotMap.BallonPresent.get());
		
		//SmartDashboard.putNumber("Angle Encodeur Bras: ", (RobotMap.encodeurBras.getRaw()* 490.0 / 360.0));
		
		SmartDashboard.putNumber("Encodeur Deplacement Droit: ", RobotMap.encodeurDeplacementDroit.getDistance());
		SmartDashboard.putNumber("Encodeur Deplacement Gauche: ", RobotMap.encodeurDeplacementGauche.getDistance());
		
		//SmartDashboard.putNumber("Deplacement PID position", (Robot.deplacement.getPosition()/SONAR_VOLTAGE2DISTANCE)*2.54);
		
		SmartDashboard.putNumber("Forward Sonar (voltage): ",     RobotMap.forwardSonar.getVoltage());
		//SmartDashboard.putNumber("Forward Sonar (value): ",       RobotMap.forwardSonar.getValue());
		//SmartDashboard.putNumber("Forward Sonar (avg voltage): ", RobotMap.forwardSonar.getAverageVoltage());
		//SmartDashboard.putNumber("Forward Sonar: (cm)",       (RobotMap.forwardSonar.getAverageVoltage()/SONAR_VOLTAGE2DISTANCE)*2.54);
		SmartDashboard.putNumber("Forward Sonar: (ft)",       (RobotMap.forwardSonar.getAverageVoltage()/SONAR_VOLTAGE2DISTANCE)*0.0833);
	
		double angle = RobotMap.gyro.getAngle();
		double Kp = 0.03;
		SmartDashboard.putNumber("Robot Gyro (rate): ", RobotMap.gyro.getRate());
		SmartDashboard.putNumber("Robot Gyro (angle): ", angle);
		
		//SmartDashboard.putNumber("X acceleration component: ", Robot.deplacement.getAccelX());
		//SmartDashboard.putNumber("Y acceleration Component: ", Robot.deplacement.getAccelY());
		//SmartDashboard.putNumber("Z acceleration Component: ", Robot.deplacement.getAccelZ());
		
		//RobotMap.drive.arcadeDrive(0.5, -angle*Kp);
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}
	
	private void log() {  //Template a developper avec les autres sous modules
        Robot.deplacement.log();    
    }
	
	public void cameraToggle(){ 
		
		    CameraServer camera0 = CameraServer.getInstance();
        	 camera0.setQuality(10);
        	 CameraServer camera1 = CameraServer.getInstance();
             camera1.setQuality(10);
             
		 
         if(Robot.oi.getJoystickCoPilote().getRawButton(11) == true){
        	
        	 camera0.startAutomaticCapture("cam0"); 
        	 }
         
          else{
           // camera0.;  
            camera1.startAutomaticCapture("cam1");
         }
		
	}
}
