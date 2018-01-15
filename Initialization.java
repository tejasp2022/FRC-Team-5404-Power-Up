/***************************************************************************************
*    Title: Gearaffes Robotics Team FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

import java.util.ArrayList;
import java.util.function.Function;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Initialization {
	// Constants
	public static final double robotDepth = 32.25; //32.25; //35.5 for last year's bot;
	public static final double robotWidth = 27.5; //27.5; //38.5 for last year's bot;

	// Global Variables
	public static char ourSwitchPosition;
	public static char scalePosition;
	public static char opposingSwitchPosition;
	public static int robotStartingPosition = 1;
	public static ArrayList<Function<Void,Boolean>> switchSequence = new ArrayList<Function<Void,Boolean>>();
	public static ArrayList<Function<Void,Boolean>> scaleSequence = new ArrayList<Function<Void,Boolean>>();

	// Autonomous Moving Powers
	public static double autoMovePower = 0.7;
	public static double autoMoveContactHigh = 0.55;
	public static double autoMoveContactLow = 0.3;
	public static double autoRotatePower = 0.5;
	
	// Joysticks
	public static Joystick driver = new Joystick(PortIO.driver);
	public static Joystick operator = new Joystick(PortIO.operator);

	// Drive Motor Controllers
	public static VictorSP FR = new VictorSP(PortIO.FR);
	public static VictorSP BR = new VictorSP(PortIO.BR);
	public static VictorSP FL = new VictorSP(PortIO.FL);
	public static VictorSP BL = new VictorSP(PortIO.BL);

	static double moveMultiplier;
	static double rotateMultiplier;
	static SpeedControllerGroup leftGroup = new SpeedControllerGroup(FL, BL);
	static SpeedControllerGroup rightGroup = new SpeedControllerGroup(FR, BR);
	static DifferentialDrive gearaffesDrive = new DifferentialDrive(leftGroup, rightGroup);
	static double move_KP = 0.06; // needs to be changed

	// Elevator Motor Controller
	public static Spark elevator = new Spark(PortIO.elv);
	static double elevateMultiplier;

	// Drive Encoders
	public static Encoder rightDriveEncoder = new Encoder(PortIO.rdEncoder1, PortIO.rdEncoder2);
	public static Encoder leftDriveEncoder = new Encoder(PortIO.ldEncoder1, PortIO.ldEncoder2);
	static final double DRIVE_TICKS_PER_REV = 213.6;//213.6; //128 for last year's bot
	static double DRIVE_INCH_PER_REV = 18.8496;
	static double DRIVE_INCHES_PER_TICK = DRIVE_INCH_PER_REV / DRIVE_TICKS_PER_REV;

	// Elevator Encoders
	public static Encoder elevatorEncoder = new Encoder(PortIO.elevatorEncoder1, PortIO.elevatorEncoder2);
	static final double ELEVATOR_INCHES_PER_TICK = 1; // needs to be changed

	// Gyros
	public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	static final double MultiplierForGyro = 1; //1.175; // needs to be changed

	// Solenoids

	// Dashboard Preferences
	public static Preferences prefs = Preferences.getInstance();

	// USB Camera
	public static CameraServer cam = CameraServer.getInstance();

	// Power Distribution Panel
	static PowerDistributionPanel pdp = new PowerDistributionPanel();
}
