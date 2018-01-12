/***************************************************************************************
*    Title: Gearaffes Robotics Team: FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
	
	public static void move(double dist, double speed) {
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
		} else {
		 Initialization.gearaffesDrive.arcadeDrive(0, 0);
		  //Initialization.rightDriveEncoder.reset();
		 //Initialization.leftDriveEncoder.reset();
		  //Initialization.gyro.reset();
		}
	}
	
	
	public static void moveUntilContact(double dist, double highSpeed, double lowSpeed) {
		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.gyro.reset();
		int successes = 0;
		while(successes<5) {
			double speed = Math.abs(Initialization.rightDriveEncoder.getDistance()) < 0.9*dist ? highSpeed : lowSpeed;
			//SmartDashboard.putNumber("GyroAngle", Initialization.gyro.getAngle());SmartDashboard.putNumber("RightEncoderDist", Initialization.rightDriveEncoder.getDistance());SmartDashboard.putNumber("LeftEncoderDist", Initialization.leftDriveEncoder.getDistance());
			Initialization.gearaffesDrive.arcadeDrive(speed, Initialization.gyro.getAngle() * Initialization.move_KP);
			if(Math.abs(Initialization.rightDriveEncoder.getRate())<=0.5){ //inches/second 
				successes++;
			}
		}
		Initialization.gearaffesDrive.arcadeDrive(0, -Initialization.gyro.getAngle() * Initialization.move_KP);
		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.gyro.reset();
	}
	
	
	public static void rotate(double angle, double power) {
		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.gyro.reset();
		while (Math.abs(Initialization.gyro.getAngle() * Initialization.MultiplierForGyro) < Math.abs(angle)) {
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
		}
		Initialization.gearaffesDrive.arcadeDrive(0, 0);
		//Initialization.left.set(0);
		//Initialization.right.set(0);

		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.gyro.reset();
	}
	
	
	public static void crossBaseline() {
		// auto line is 10 feet from the alliance wall, so move forward 10.5 feet = 126 inches
		move(126, 0.45);
	}
	
	
	public static void placeCubeOnSwitch() {
		if(Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition ==1) {
			move(168 - Initialization.robotDepth/2, 0.45); //Move Forward 168 Inches - (robot depth/2)
			rotate(90,0.45); //Rotate Right 90 degrees
			moveUntilContact(55.56 - Initialization.robotWidth/2, 0.45, 0.2); //Move Forward Until Contact -> 55.56 Inches - (robot width/2)
		
		} else if(Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition ==2) {
			move(59-Initialization.robotDepth, 0.45); //Move Forward 59 Inches - robot depth
			rotate(-90, 0.45); //Rotate Left 90 degrees
			move(42+Initialization.robotWidth/2, 0.45); //Move Forward 42 Inches + (robot width/2)
			rotate(90, 0.45); //Rotate Right 90 degrees
			moveUntilContact(81, 0.45, 0.2); //Move Forward Until Contact -> 81 Inches
		
		} else if(Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition ==3) {
			move(228-Initialization.robotWidth/2,0.45); //Move Forward 228 Inches - (robot width/2)
			rotate(-90, 0.45); //Rotate Left 90 degrees
			move(264-Initialization.robotWidth, 0.45); //Move Forward 264 Inches - robot width
			rotate(-90, 0.45); //Rotate Left 90 degrees
			move(60,0.45); //Move Forward 60 inches
			rotate(-90, 0.45); //Rotate Left 90 degrees
			moveUntilContact(55.56 - Initialization.robotWidth/2, 0.45, 0.2); //Move Forward Until Contact -> 55.56 Inches - (robot width/2)
		
		} else if(Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition ==1) {
			move(228-Initialization.robotWidth/2,0.45); //Move Forward 228 Inches - (robot width/2)
			rotate(90, 0.45); //Rotate Right 90 degrees
			move(264-Initialization.robotWidth, 0.45); //Move Forward 264 Inches - robot width
			rotate(90, 0.45); //Rotate Right 90 degrees
			move(60,0.45); //Move Forward 60 inches
			rotate(90, 0.45); //Rotate Right 90 degrees
			moveUntilContact(55.56 - Initialization.robotWidth/2, 0.45, 0.2); //Move Forward Until Contact -> 55.56 Inches - (robot width/2)
		
		} else if(Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition ==2) {
			move(59-Initialization.robotDepth, 0.45); //Move Forward 59 Inches - robot depth
			rotate(90, 0.45); //Rotate Right 90 degrees
			move(66-Initialization.robotWidth/2, 0.45); //Move Forward 66 Inches - (robot width/2)
			rotate(-90, 0.45); //Rotate Left 90 degrees
			moveUntilContact(81, 0.45, 0.2); //Move Forward Until Contact -> 81 Inches
		
		} else if(Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition ==3) {
			move(168 - Initialization.robotDepth/2, 0.45); //Move Forward 168 Inches - (robot depth/2)
			rotate(-90,0.45); //Rotate Left 90 degrees
			moveUntilContact(55.56 - Initialization.robotWidth/2, 0.45, 0.2); //Move Forward Until Contact -> 55.56 Inches - (robot width/2)
		}
	}
	
	
	public static void placeCubeOnScale() {
		if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition == 1) {

		} else if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition == 2) {

		} else if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition == 3) {

		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition == 1) {

		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition == 2) {

		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition == 3) {

		}
	}
	
	public static void elevatorToBase() {
		int successes = 0;
		while (successes < 5) {
			Initialization.elevator.set(-0.45);
			if (Math.abs(Initialization.elevatorEncoder.getRate()) <= 0.1) { // inches/second
				successes++;
			}
		}
		Initialization.elevator.set(0);
		Initialization.elevatorEncoder.reset();
	}
	
	public static void setElevatorHeight(double height) {
		double dist = height - Math.abs(Initialization.elevatorEncoder.getDistance());
		if (dist == -Math.abs(Initialization.elevatorEncoder.getDistance())) {
			elevatorToBase();
		} else if (dist > 0) {
			while (Math.abs(Initialization.elevatorEncoder.getDistance()) < height) {
				Initialization.elevator.set(0.45); // assuming positive speed moves the elevator up
			}
			Initialization.elevator.set(0);
		} else if (dist < 0) {
			while (Math.abs(Initialization.elevatorEncoder.getDistance()) > height) {
				Initialization.elevator.set(-0.45); // assuming negative speed moves the elevator down
			}
			Initialization.elevator.set(0);
		}
	}	
}
