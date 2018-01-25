/***************************************************************************************
*    Title: Gearaffes Robotics Team FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/                      
package org.usfirst.frc.team5404.robot;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

	public static int autoProcess = 0;
	
	public void robotInit() {
		resetSensors();
		calibrateSensors();
	}
	
	public void autonomousInit() {
		Autonomous.getMatchData();
		Autonomous.determineAutonomousSequence();
		assignPreferenceVariables();
		autoProcess = 0;
		resetSensors();
		calibrateSensors();
		Initialization.gearaffesPID.enable();
	}

	boolean moveCalled = false;
	public void autonomousPeriodic() {
		if(!moveCalled) {
			if(Autonomous.move(120, 0.7)) {
			moveCalled = true;
			}
		}
		
	}

	public void teleopInit() {
		resetSensors();
		calibrateSensors();
	}
	
	public void teleopPeriodic() {
		Teleop.drive();
		Teleop.elevate();
		Teleop.endGameRumble();
		//Teleop.climb();
	}

	public void disabledInit() {
		
	}
	
	public void disabledPeriodic() {
		
	}
	
	public static void resetSensors() {
		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.elevatorEncoder.reset();
		Initialization.gyro.reset();
	}
	public void calibrateSensors() {
		Initialization.elevatorEncoder.setDistancePerPulse(Initialization.ELEVATOR_INCHES_PER_TICK);
		Initialization.rightDriveEncoder.setDistancePerPulse(Initialization.DRIVE_INCHES_PER_TICK);
		Initialization.leftDriveEncoder.setDistancePerPulse(Initialization.DRIVE_INCHES_PER_TICK);
		Initialization.elevatorEncoder.setSamplesToAverage(5);
		Initialization.rightDriveEncoder.setSamplesToAverage(5);
		Initialization.leftDriveEncoder.setSamplesToAverage(5);
	}
	
	public void setAutonFalse() {
		
	}
	
	public static double formatValue(double value) {
		return Double.valueOf(new DecimalFormat("###.00").format(value));
	}
	
	public static void displaySensors() {
		SmartDashboard.putNumber("Left Encoder Distance", formatValue(Math.abs(Initialization.leftDriveEncoder.getDistance())));
		SmartDashboard.putNumber("Right Encoder Distance", formatValue(Math.abs(Initialization.rightDriveEncoder.getDistance())));
		SmartDashboard.putNumber("Gyro Angle", formatValue(Initialization.gyro.getAngle()));
	}
	
	public static void assignPreferenceVariables() {
		// Autonomous Moving Speeds
		Initialization.autoMoveSpeed = Initialization.prefs.getDouble("Auto Move Speed", 70)/100;
		Initialization.autoMoveContactHigh = Initialization.prefs.getDouble("Auto Move Contact High", 55)/100;	
		Initialization.autoMoveContactLow = Initialization.prefs.getDouble("Auto Move Contact Low", 30)/100;					
		Initialization.autoRotateSpeed = Initialization.prefs.getDouble("Auto Rotate Speed", 50)/100;					
		
		// Automation Moving Speeds
		Initialization.automationHighSpeed = Initialization.prefs.getDouble("Automation High Speed", 70)/100;
		Initialization.automationBottomSpeed = Initialization.prefs.getDouble("Automation Bottom Speed", 10)/100;
		Initialization.automationTopSpeed = Initialization.prefs.getDouble("Automation Top Speed", 40)/100;
		
		// Robot Starting Position
		Initialization.robotStartingPosition = Initialization.prefs.getInt("Robot Starting Position", 1);
		
		// Teleop Multipliers
		Initialization.moveMultiplier = Initialization.prefs.getDouble("Move Multiplier", 70)/100;
		Initialization.rotateMultiplier = Initialization.prefs.getDouble("Rotate Multiplier", 50)/100;
		Initialization.elevateMultiplier = Initialization.prefs.getDouble("Elevate Multiplier", 70)/100;
		
		// PID Multipliers
		Initialization.move_KP = Initialization.prefs.getDouble("Move KP", 0.06);
		Initialization.move_KI = Initialization.prefs.getDouble("Move KI", 0.03);
	}
}