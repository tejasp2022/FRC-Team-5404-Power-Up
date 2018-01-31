/***************************************************************************************
*    Title: Gearaffes Robotics Team FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Initialization {
	// Robot Dimensions
	public static final double robotDepth = 32.25;      
	public static final double robotWidth = 27.5;  

	//Autonomous Switch or Scale
	public static String autoStrat;
	
	// Autonomous Positioning Variables
	public static char ourSwitchPosition;
	public static char scalePosition;
	public static char opposingSwitchPosition;
	public static double robotStartingPosition;
	public static ArrayList<Function<Void,Boolean>> switchSequence = new ArrayList<Function<Void,Boolean>>();
	public static ArrayList<Function<Void,Boolean>> scaleSequence = new ArrayList<Function<Void,Boolean>>();

	// Test Sequence
	public static ArrayList<Function<Void,Boolean>> testSequence = new ArrayList<Function<Void,Boolean>>();
	
	// Autonomous Moving Speeds
	public static double autoDelayTime;
	public static double autoMoveSpeed;
	public static double autoMoveContactHigh;
	public static double autoMoveContactLow;
	public static double autoRotateSpeed;
	
	// Automation Moving Speeds
	public static double automationHighSpeed;
	public static double automationBottomSpeed;
	public static double automationTopSpeed;
	
	// Joysticks
	public static Joystick driver = new Joystick(PortIO.driver);
	public static Joystick operator = new Joystick(PortIO.operator);

	// Drive Motor Controllers
	public static VictorSP FR = new VictorSP(PortIO.FR);
	public static VictorSP BR = new VictorSP(PortIO.BR);
	public static VictorSP FL = new VictorSP(PortIO.FL);
	public static VictorSP BL = new VictorSP(PortIO.BL);

	// Motor Controller Variables
	static double moveMultiplier;
	static double rotateMultiplier;
	static SpeedControllerGroup leftGroup = new SpeedControllerGroup(FL, BL);
	static SpeedControllerGroup rightGroup = new SpeedControllerGroup(FR, BR);
	static DifferentialDrive gearaffesDrive = new DifferentialDrive(leftGroup, rightGroup);
	
	static double move_KP;
	static double move_KI;
	
	// Elevator Motor Controller
	public static Spark elevator = new Spark(PortIO.elevator);
	static double elevateMultiplier;

	// Drive Encoders
	public static Encoder rightDriveEncoder = new Encoder(PortIO.rdEncoder1, PortIO.rdEncoder2);
	public static Encoder leftDriveEncoder = new Encoder(PortIO.ldEncoder1, PortIO.ldEncoder2);
	static final double DRIVE_TICKS_PER_REV = 213.6; 
	static double DRIVE_INCH_PER_REV = 18.8496;                                                                                  
	static double DRIVE_INCHES_PER_TICK = DRIVE_INCH_PER_REV / DRIVE_TICKS_PER_REV; //0.08824719

	// Motor-encoder pairs for test sequence
	public static List<Test.SmartController> controllerEncoderPairs;
	
	// Elevator Encoders
	public static Encoder elevatorEncoder = new Encoder(PortIO.elevatorEncoder1, PortIO.elevatorEncoder2);
	public static final double ELEVATOR_INCHES_PER_TWO_FEET = 24;
	public static final double ELEVATOR_TICKS_PER_TWO_FEET = 10732.2526551;
	static final double ELEVATOR_INCHES_PER_TICK = ELEVATOR_INCHES_PER_TWO_FEET/ELEVATOR_TICKS_PER_TWO_FEET; //0.00223625; 
	
	// Limit Switches
	public static DigitalInput bottomLimitSwitch = new DigitalInput(PortIO.bottomLimitSwitch);
	public static DigitalInput topLimitSwitch = new DigitalInput(PortIO.topLimitSwitch);
	

	// Gyros
	public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	static final double MultiplierForGyro = 1;
	
	// Range Finder
	public static AnalogInput rangeFinder = new AnalogInput(PortIO.rangeFinder);
	
	//Other ultrasonic range finder
	//public static AnalogInput us = new AnalogInput(PortIO.us);
	public static DigitalOutput usTeach = new DigitalOutput(PortIO.usTeach);
	//color mark sensor
	//public static DigitalInput colorMark = new DigitalInput(PortIO.colorMark);

	// Solenoids

	// PID
	static GearaffesPID gearaffesPID = new GearaffesPID(move_KP, move_KI, gyro, new GearaffesPID.GearaffesOutput());
	
	
	// Dashboard Preferences
	public static Preferences prefs = Preferences.getInstance();

	// USB Camera
	public static CameraServer cam = CameraServer.getInstance();

	// Power Distribution Panel
	static PowerDistributionPanel pdp = new PowerDistributionPanel();
}