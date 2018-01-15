/***************************************************************************************
*    Title: Gearaffes Robotics Team FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Teleop {
	public static void drive(){
		Initialization.gearaffesDrive.arcadeDrive(-Initialization.moveMultiplier * Math.pow(Initialization.driver.getRawAxis(1), 2), Initialization.rotateMultiplier * Math.pow(Initialization.driver.getRawAxis(4), 2), false);
		SmartDashboard.putNumber("Robot Speedometer", Robot.formatValue(Math.abs(Initialization.leftDriveEncoder.getRate())/12));
	}
	
	public static void elevate() {
		Initialization.elevator.set(Initialization.elevateMultiplier * Math.pow(Initialization.operator.getRawAxis(1), 2));
		SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance())/12));
		if (Math.abs(Initialization.elevatorEncoder.getRate()) <= 0.1 && Math.abs(Initialization.elevator.get())>0) {
			Initialization.elevatorEncoder.reset();
		}
	}
	
	public static void climb(){
		// Climbing Code Here
	}
}
