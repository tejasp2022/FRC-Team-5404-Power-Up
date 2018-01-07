/***************************************************************************************
*    Title: Gearaffes Robotics Team: FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/                      


package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

	@Override
	public void robotInit() {
		resetSensors();
		calibrateSensors();
	}
	
	@Override
	public void autonomousInit() {
		getMatchData();
		resetSensors();
		calibrateSensors();
	}

	@Override
	public void autonomousPeriodic() {

	}

	public void teleopInit() {
		resetSensors();
		calibrateSensors();
	}
	
	@Override
	public void teleopPeriodic() {
	
	}

	public void disabledInit() {
		resetSensors();
		calibrateSensors();
	}
	
	public void disabledPeriodic() {
		
	}
	
	public void resetSensors() {
		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.gyro.reset();
	}
	public void calibrateSensors() {
		Initialization.rightDriveEncoder.setDistancePerPulse(Initialization.INCHES_PER_TICK);
		Initialization.leftDriveEncoder.setDistancePerPulse(Initialization.INCHES_PER_TICK);
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
	
}
