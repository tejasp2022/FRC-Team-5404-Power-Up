/***************************************************************************************
*    Title: Gearaffes Robotics Team: FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Initialization {
	public static char ourSwitchPosition;
	public static char scalePosition;
	public static char opposingSwitchPosition;
	public static int robotStartingPosition;
	
	public static Joystick driver = new Joystick(PortIO.driver);
	public static Joystick operator = new Joystick(PortIO.operator);
	
	//Drive Motor Controllers
	public static VictorSP left = new VictorSP(PortIO.left);
	public static VictorSP right = new VictorSP(PortIO.right);
	
	static DifferentialDrive gearaffesDrive = new DifferentialDrive(left, right);
	static double move_KP = 0.06;
	
	//Encoders
	public static Encoder rightDriveEncoder = new Encoder(PortIO.rdEncoder1,PortIO.rdEncoder2);
	public static Encoder leftDriveEncoder = new Encoder(PortIO.ldEncoder1,PortIO.ldEncoder2);
	static final double DRIVE_TICKS_PER_REV = 128; // needs to be changed
	static double INCH_PER_REV = 18.8496; //needs to be changed
	static double INCHES_PER_TICK = INCH_PER_REV / DRIVE_TICKS_PER_REV;
	
	//Gyros
	public static ADXRS450_Gyro gyro= new ADXRS450_Gyro();
	static final double MultiplierForGyro = 1.175;
	
	//Solenoids
	
	// Dashboard Preferences
	public static Preferences prefs = Preferences.getInstance();
	
	// USB Camera
	public static CameraServer cam = CameraServer.getInstance();
		
	// Power Distribution Panel
	static PowerDistributionPanel pdp = new PowerDistributionPanel();
}
