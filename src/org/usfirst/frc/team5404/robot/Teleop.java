/***************************************************************************************
*    Title: Gearaffes Robotics Team: FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Teleop {
	public static void drive(){
		Initialization.gearaffesDrive.arcadeDrive(-Initialization.moveMultiplier * Initialization.driver.getRawAxis(1), Initialization.rotateMultiplier * Initialization.driver.getRawAxis(4), true);
		SmartDashboard.putNumber("Robot Speed (FPS)", Robot.formatNumber(Math.abs(Initialization.rightDriveEncoder.getRate())/12));
	}
	
	public static void elevate() {
		Initialization.elevator.set(Initialization.elevateMultiplier * Math.pow(Initialization.operator.getRawAxis(1), 2));
		SmartDashboard.putNumber("Elevator Height (Feet)", Robot.formatNumber(Math.abs(Initialization.elevatorEncoder.getDistance())/12));
	}
	
	public static void climb(){
		
	}
	
	public static void diagnosticData() {
		Initialization.pdp.getVoltage();
		Initialization.pdp.getTotalCurrent();
		Initialization.pdp.getTotalPower();
		Initialization.pdp.getTotalEnergy();
		Initialization.pdp.getTemperature();
		double[] currentByChannel = new double[15];
		for(int i=0; i<15; i++) {
			currentByChannel[i] = Initialization.pdp.getCurrent(i+1);
		}
	}
}