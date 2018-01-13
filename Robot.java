/***************************************************************************************
*    Title: Gearaffes Robotics Team: FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/                      


package org.usfirst.frc.team5404.robot;

import java.text.DecimalFormat;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;

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
		//Teleop.elevate();
		//Teleop.climb();
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
		Initialization.rightDriveEncoder.setSamplesToAverage(1);
		Initialization.leftDriveEncoder.setSamplesToAverage(1);
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
	
}
