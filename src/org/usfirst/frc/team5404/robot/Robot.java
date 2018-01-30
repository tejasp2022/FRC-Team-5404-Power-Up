/***************************************************************************************
*    Title: Gearaffes Robotics Team FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/                      
package org.usfirst.frc.team5404.robot;

import java.text.DecimalFormat;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	public static int autoProcess = 0;
	
	public void robotInit() {
		Initialization.gearaffesDrive.setSafetyEnabled(false);
		resetSensors();
		calibrateSensors();
	}
	
	public void autonomousInit() {
		Initialization.gearaffesDrive.setSafetyEnabled(false);
		assignPreferenceVariables();
		Autonomous.getMatchData();
		Autonomous.determineAutonomousSequence();
		autoProcess = 0;
		resetSensors();
		calibrateSensors();
		Initialization.gearaffesPID = new GearaffesPID(Initialization.move_KP, Initialization.move_KI, Initialization.gyro, new GearaffesPID.GearaffesOutput());
		Initialization.gearaffesPID.enable();
	}

	public void autonomousPeriodic() {
		if(Initialization.autoStrat.trim().equalsIgnoreCase("scale")) {
			Autonomous.placeCubeOnScale();
		} else if (Initialization.autoStrat.trim().equalsIgnoreCase("switch")) {
			Autonomous.placeCubeOnSwitch();
		} else {
			SmartDashboard.putString("Autonomous Error Alert", "A Valid Autonomous Strategy Was Not Selected");
		}
	}

	public void teleopInit() {
		Initialization.gearaffesDrive.setSafetyEnabled(false);
		assignPreferenceVariables();
		resetSensors();
		calibrateSensors();
	}
	
	public void teleopPeriodic() {
		Teleop.drive();
		Teleop.elevate();
		Teleop.endGameRumble();
		Teleop.rangeDistance();
		Teleop.elevatorRumble();
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
		Initialization.rightDriveEncoder.setSamplesToAverage(1);
		Initialization.leftDriveEncoder.setSamplesToAverage(1);
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
		Initialization.autoStrat = Initialization.prefs.getString("Autonomous Strategy", "Not Found");
		
		// Autonomous Moving Speeds
		Initialization.autoMoveSpeed = Initialization.prefs.getDouble("Auto Move Speed", 70)/100;
		Initialization.autoMoveContactHigh = Initialization.prefs.getDouble("Auto Move Contact High", 55)/100;	
		Initialization.autoMoveContactLow = Initialization.prefs.getDouble("Auto Move Contact Low", 30)/100;					
		Initialization.autoRotateSpeed = Initialization.prefs.getDouble("Auto Rotate Speed", 50)/100;					
		
		// Automation Moving Speeds
		Initialization.autoDelayTime = Initialization.prefs.getDouble("Autonomous Delay", 0);
		Initialization.automationHighSpeed = Initialization.prefs.getDouble("Automation High Speed", 70)/100;
		Initialization.automationTopSpeed = Initialization.prefs.getDouble("Automation Top Speed", 40)/100;
		Initialization.automationBottomSpeed = Initialization.prefs.getDouble("Automation Bottom Speed", 10)/100;
		
		// Robot Starting Position
		Initialization.robotStartingPosition = Initialization.prefs.getDouble("Robot Starting Position", 1);
		
		// Teleop Multipliers
		Initialization.moveMultiplier = Initialization.prefs.getDouble("Move Multiplier", 70)/100;
		Initialization.rotateMultiplier = Initialization.prefs.getDouble("Rotate Multiplier", 50)/100;
		Initialization.elevateMultiplier = Initialization.prefs.getDouble("Elevate Multiplier", 70)/100;
		
		// PID Multipliers
		Initialization.move_KP = Initialization.prefs.getDouble("Move KP", 0.06);
		Initialization.move_KI = Initialization.prefs.getDouble("Move KI", 0.03);
	}
}