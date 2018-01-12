/***************************************************************************************
*    Title: Gearaffes Robotics Team: FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

public class Teleop {
	public static void drive(){
		Initialization.gearaffesDrive.arcadeDrive(-Initialization.movePower * Initialization.driver.getRawAxis(1), Initialization.rotationPower * Initialization.driver.getRawAxis(4), true);
	}
	
	public static void climb(){
		
	}
	
	public static void elevator() {
		Initialization.elevator.set(Initialization.operator.getRawAxis(1));
	}
}
