/***************************************************************************************
*    Title: Gearaffes Robotics Team FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Teleop {
	/**
	 * Drives the robot using input from the driver's joystick
	 */
	public static void drive(){
		double finalMoveMultiplier = Initialization.operator.getRawButton(0) ? 1 : (Initialization.operator.getRawButton(1) ? 0.5 : Initialization.moveMultiplier);
		double finalRotateMultiplier = Initialization.operator.getRawButton(0) ? 1 : (Initialization.operator.getRawButton(1) ? 0.5 : Initialization.rotateMultiplier);
		Initialization.gearaffesDrive.arcadeDrive(Math.signum(Initialization.driver.getRawAxis(1))* -finalMoveMultiplier * Math.pow(Initialization.driver.getRawAxis(1), 2), Math.signum(Initialization.driver.getRawAxis(4)) * finalRotateMultiplier * Math.pow(Initialization.driver.getRawAxis(4), 2), false);
		SmartDashboard.putNumber("Robot Speedometer", Robot.formatValue(Math.abs(Initialization.leftDriveEncoder.getRate())/12));
	}
	
	/**
	 * Controls the elevator using input from the operator's joystick
	 */
	static boolean processInProgress = false;
	static double elevatorTargetHeight, elevatorSpeed;
	public static void elevate() {	
		if(processInProgress) {
			Autonomous.setElevatorHeight(elevatorTargetHeight, elevatorSpeed);
		} else if(Initialization.operator.getRawButtonPressed(0)) {
			processInProgress = true;
			elevatorTargetHeight = 24;
			elevatorSpeed = 0.7;
				
		} else if (Initialization.operator.getRawButtonPressed(1) ) {
			processInProgress = true;
			elevatorTargetHeight = 36;
			elevatorSpeed = 0.7;
							
		} else if (Initialization.operator.getRawButtonPressed(2) ) {
			processInProgress = true;
			elevatorTargetHeight = 48;
			elevatorSpeed = 0.7;
							
		} else if (Initialization.operator.getRawButtonPressed(2) ) {
			processInProgress = true;
			elevatorTargetHeight = 60;
			elevatorSpeed = 0.7;
							
		} else {
			Initialization.elevator.set(Initialization.elevateMultiplier * Math.pow(Initialization.operator.getRawAxis(1), 2));
			if (Math.abs(Initialization.elevatorEncoder.getRate()) <= 0.1 && Math.abs(Initialization.elevator.get())>0) {
				Initialization.elevatorEncoder.reset();
			}
		}
		SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance())/12));
	}
	
	/**
	 * Controls the climber using input from the operator's joystick
	 */
	public static void climb(){
		// Climbing Code Here
	}
	
	public static void rumble(){
		if((Timer.getMatchTime()<30 && Timer.getMatchTime()>29.5) || (Timer.getMatchTime()<29 && Timer.getMatchTime()>28.5) || (Timer.getMatchTime()<28 && Timer.getMatchTime()>27.5)){
			Initialization.driver.setRumble(RumbleType.kLeftRumble, 1);
			Initialization.driver.setRumble(RumbleType.kRightRumble, 1);
			Initialization.operator.setRumble(RumbleType.kLeftRumble, 1);
			Initialization.operator.setRumble(RumbleType.kRightRumble, 1);
		} else {
			Initialization.driver.setRumble(RumbleType.kLeftRumble, 0);
			Initialization.driver.setRumble(RumbleType.kRightRumble, 0);
			Initialization.operator.setRumble(RumbleType.kLeftRumble, 0);
			Initialization.operator.setRumble(RumbleType.kRightRumble, 0);
		}
	}
}
