/***************************************************************************************
*    Title: Gearaffes Robotics Team FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/                      
package org.usfirst.frc.team5404.robot;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.DriverStation;
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
		autoProcess = 0;
		resetSensors();
		calibrateSensors();
	}

	public void autonomousPeriodic() {
		Autonomous.placeCubeOnScale();
	}

	public void teleopInit() {
		resetSensors();
		calibrateSensors();
		Initialization.moveMultiplier = Initialization.prefs.getDouble("Move Multiplier", 70)/100;
		Initialization.rotateMultiplier = Initialization.prefs.getDouble("Rotate Multiplier", 70)/100;
		Initialization.elevateMultiplier = Initialization.prefs.getDouble("Elevate Multiplier", 70)/100;
	}
	
	public void teleopPeriodic() {
		Teleop.drive();
		//Teleop.elevate();
		//Teleop.climb();
	}

	public void disabledInit() {
		
	}
	
	public void disabledPeriodic() {
		
	}
	
	public static void resetSensors() {
		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.gyro.reset();
	}
	public void calibrateSensors() {
		Initialization.elevatorEncoder.setDistancePerPulse(Initialization.ELEVATOR_INCHES_PER_TICK);
		Initialization.rightDriveEncoder.setDistancePerPulse(Initialization.DRIVE_INCHES_PER_TICK);
		Initialization.leftDriveEncoder.setDistancePerPulse(Initialization.DRIVE_INCHES_PER_TICK);
		Initialization.elevatorEncoder.setSamplesToAverage(1);
		Initialization.rightDriveEncoder.setSamplesToAverage(10);
		Initialization.leftDriveEncoder.setSamplesToAverage(10);
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
	
}