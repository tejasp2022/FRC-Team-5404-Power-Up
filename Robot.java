/***************************************************************************************
*    Title: Gearaffes Robotics Team: FIRST Robotics Competition 2018 Code
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
	
	@Override
	public void robotInit() {
		resetSensors();
		calibrateSensors();
		Initialization.gearaffesDrive.setSafetyEnabled(false);
	}
	
	@Override
	public void autonomousInit() {
		Autonomous.determineAutonomousSequence();
		autoProcess = 0;
		getMatchData();
		resetSensors();
		calibrateSensors();
		
		
	}

	boolean hasRun = false;
	@Override
	public void autonomousPeriodic() {
		/*if (!hasRun) {
			Autonomous.crossBaseline();
			hasRun = false;
		}*/
		Autonomous.placeCubeOnSwitch();
		Teleop.diagnosticData();
	}

	public void teleopInit() {
		resetSensors();
		calibrateSensors();
		Initialization.moveMultiplier = Initialization.prefs.getDouble("Move Multiplier", 70)/100;
		Initialization.rotateMultiplier = Initialization.prefs.getDouble("Rotate Multiplier", 70)/100;
		Initialization.elevateMultiplier = Initialization.prefs.getDouble("Elevate Multiplier", 70)/100;
	}
	
	@Override
	public void teleopPeriodic() {
		Teleop.drive();
		displaySensors();
		//Teleop.elevate();
		//Teleop.climb();
		//Teleop.diagnosticData();
	}

	public void disabledInit() {
		resetSensors();
		calibrateSensors();
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
	
	public static void getMatchData() {
		 try{
			 Initialization.ourSwitchPosition = DriverStation.getInstance().getGameSpecificMessage().charAt(0); // either L or R
			 Initialization.scalePosition = DriverStation.getInstance().getGameSpecificMessage().charAt(1);
			 Initialization.opposingSwitchPosition = DriverStation.getInstance().getGameSpecificMessage().charAt(2);
		 }catch(NullPointerException e) {
			 System.err.println("One or more of the field element positions could not be determined");
		 }
	}
	
	public static double formatNumber(double value) {
		return Double.valueOf(new DecimalFormat("###.00").format(value));
	}
	
	public static void displaySensors() {
		SmartDashboard.putNumber("Left Encoder Distance", formatNumber(Initialization.leftDriveEncoder.getDistance()));
		SmartDashboard.putNumber("Right Encoder Distance", formatNumber(Initialization.rightDriveEncoder.getDistance()));
		SmartDashboard.putNumber("Gyro Angle", formatNumber(Initialization.gyro.getAngle()));
	}
	
}