/***************************************************************************************
*    Title: Gearaffes Robotics Team: FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

import java.util.ArrayList;
import java.util.function.Function;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
	static ArrayList<Function<Void,Boolean>> funcyList = new ArrayList<Function<Void,Boolean>>();
	static ArrayList<Function<Void,Boolean>> funcyListScale = new ArrayList<Function<Void,Boolean>>();
	static int successes = 0;
	
	public static boolean move(double dist, double speed) {
		//Initialization.rightDriveEncoder.reset();
		//Initialization.leftDriveEncoder.reset();
		//Initialization.gyro.reset();
		if (Math.abs(Initialization.rightDriveEncoder.getDistance()) < Math.abs(dist)) {
			//SmartDashboard.putNumber("GyroAngle", Initialization.gyro.getAngle());SmartDashboard.putNumber("RightEncoderDist", Initialization.rightDriveEncoder.getDistance());SmartDashboard.putNumber("LeftEncoderDist", Initialization.leftDriveEncoder.getDistance());
			if (dist > 0) {
				Initialization.gearaffesDrive.arcadeDrive(speed, Initialization.gyro.getAngle() * Initialization.move_KP);
			} else {
				Initialization.gearaffesDrive.arcadeDrive(-speed, Initialization.gyro.getAngle() * Initialization.move_KP);
			}
			return true;
		} else {
		 Initialization.gearaffesDrive.arcadeDrive(0, 0);
		 return false;
		 //Initialization.rightDriveEncoder.reset();
		 //Initialization.leftDriveEncoder.reset();
		 //Initialization.gyro.reset();
		}
	}
	
	/*public static void move(double dist, double speed) {
		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.gyro.reset();
		while (Math.abs(Initialization.rightDriveEncoder.getDistance()) < Math.abs(dist)) {
		 	//SmartDashboard.putNumber("GyroAngle", Initialization.gyro.getAngle());SmartDashboard.putNumber("RightEncoderDist", Initialization.rightDriveEncoder.getDistance());SmartDashboard.putNumber("LeftEncoderDist", Initialization.leftDriveEncoder.getDistance());
			if (dist > 0) {
				Initialization.gearaffesDrive.arcadeDrive(speed, Initialization.gyro.getAngle() * Initialization.move_KP);
			} else {
				Initialization.gearaffesDrive.arcadeDrive(-speed, Initialization.gyro.getAngle() * Initialization.move_KP);
			}
		}
		Initialization.gearaffesDrive.arcadeDrive(0, -Initialization.gyro.getAngle() * Initialization.move_KP);
		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.gyro.reset();
	}*/
	
	public static boolean moveUntilContact(double dist, double highSpeed, double lowSpeed) {
		
		if(successes<5) {
			double speed = Math.abs(Initialization.rightDriveEncoder.getDistance()) < 0.9*dist ? highSpeed : lowSpeed;
			//SmartDashboard.putNumber("GyroAngle", Initialization.gyro.getAngle());SmartDashboard.putNumber("RightEncoderDist", Initialization.rightDriveEncoder.getDistance());SmartDashboard.putNumber("LeftEncoderDist", Initialization.leftDriveEncoder.getDistance());
			Initialization.gearaffesDrive.arcadeDrive(speed, Initialization.gyro.getAngle() * Initialization.move_KP);
			if(Math.abs(Initialization.rightDriveEncoder.getRate())<=0.5){ //inches/second 
				successes++;
			}
			return false;
		}
		Initialization.gearaffesDrive.arcadeDrive(0, 0);
		return true;
	}
	
	
	public static boolean rotate(double angle, double power) {
		if (Math.abs(Initialization.gyro.getAngle() * Initialization.MultiplierForGyro) < Math.abs(angle)) {
			SmartDashboard.putNumber("GyroAngle", Initialization.gyro.getAngle());
			if (angle > 0) {
				Initialization.gearaffesDrive.arcadeDrive(0, power);
				//Initialization.left.set(1 * power);
				//Initialization.right.set(1 * power);
				
			} else {
				Initialization.gearaffesDrive.arcadeDrive(0, -power);
				//Initialization.left.set(-1 * power);
				//Initialization.right.set(-1 * power);
			}
			return true;
		} else {
			Initialization.gearaffesDrive.arcadeDrive(0, 0);
			return false;
		}
		//Initialization.left.set(0);
		//Initialization.right.set(0);

	}
	
	
	public static void crossBaseline() {
		// auto line is 10 feet from the alliance wall, so move forward 10.5 feet = 126 inches
		move(126, 0.45);
	}
	
	
	public static void placeCubeOnSwitch() {
		//new stuff using ArrayList of Functions
		if(Robot.autoProcess < funcyList.size()) {
			if(funcyList.get(Robot.autoProcess).apply(null)) {
				Robot.resetSensors();
				Robot.autoProcess++;
			}
		}
	}
	
	
	public static void placeCubeOnScale() {
		if(Robot.autoProcess < funcyListScale.size()) {
			if(funcyListScale.get(Robot.autoProcess).apply(null)) {
				Robot.resetSensors();
				Robot.autoProcess++;
			}
		}
	}
	
	public static void elevatorToBase(double speed) {
		int successes = 0;
		while (successes < 5) {
			Initialization.elevator.set(-speed); // assuming negative speed moves the elevator down
			if (Math.abs(Initialization.elevatorEncoder.getRate()) <= 0.1) { // inches/second
				successes++;
			}
		}
		Initialization.elevator.set(0);
		Initialization.elevatorEncoder.reset();
	}
	
	public static void setElevatorHeight(double height, double speed) {
		double dist = height - Math.abs(Initialization.elevatorEncoder.getDistance());
		if (dist == -Math.abs(Initialization.elevatorEncoder.getDistance())) {
			elevatorToBase(speed);
		} else if (dist > 0) {
			while (Math.abs(Initialization.elevatorEncoder.getDistance()) < height) {
				Initialization.elevator.set(speed); // assuming positive speed moves the elevator up
			}
			Initialization.elevator.set(0);
		} else if (dist < 0) {
			while (Math.abs(Initialization.elevatorEncoder.getDistance()) > height) {
				Initialization.elevator.set(-speed); // assuming negative speed moves the elevator down
			}
			Initialization.elevator.set(0);
		}
	}
}
