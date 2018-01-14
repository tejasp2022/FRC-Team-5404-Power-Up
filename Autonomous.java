/***************************************************************************************
*    Title: Gearaffes Robotics Team: FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

public class Autonomous {

	
	public static boolean crossBaseline() {
		return move(126, 0.45);
	}
	
	// Autonomous Routines
	public static void placeCubeOnSwitch() {
		if(Robot.autoProcess < Initialization.switchSequence.size()) {
			if(Initialization.switchSequence.get(Robot.autoProcess).apply(null)) {
				Robot.resetSensors();
				Robot.autoProcess++;
			}
		}
	}
	
	public static void placeCubeOnScale() {
		if(Robot.autoProcess < Initialization.scaleSequence.size()) {
			if(Initialization.scaleSequence.get(Robot.autoProcess).apply(null)) {
				Robot.resetSensors();
				Robot.autoProcess++;
			}
		}
	}
	
	//building blocks
	
	public static boolean move(double dist, double speed) {
		if (Math.abs(Initialization.leftDriveEncoder.getDistance()) < Math.abs(dist)) {
			if (dist > 0) {
				Initialization.gearaffesDrive.arcadeDrive(speed, -Initialization.gyro.getAngle() * Initialization.move_KP);
			} else {
				Initialization.gearaffesDrive.arcadeDrive(-speed, -Initialization.gyro.getAngle() * Initialization.move_KP);
			}
			Robot.displaySensors();
			return false;
		} else {
		 Initialization.gearaffesDrive.arcadeDrive(0, 0);
		 return true;
		}
	}
	
	public static int successes = 0;
	public static boolean moveUntilContact(double dist, double highSpeed, double lowSpeed) {
		Robot.displaySensors();
		if(successes<5) {
			double speed = Math.abs(Initialization.leftDriveEncoder.getDistance()) < 0.8*dist ? highSpeed : lowSpeed;
			Initialization.gearaffesDrive.arcadeDrive(speed, -Initialization.gyro.getAngle() * Initialization.move_KP);
			if(Math.abs(Initialization.leftDriveEncoder.getRate())<=0.5){ //inches/second 
				successes++;
			}
			return false;
		} else {
			Initialization.gearaffesDrive.arcadeDrive(0, 0);
			return true;
		}
	}
	
	
	public static boolean rotate(double angle, double power) {
		if (Math.abs(Initialization.gyro.getAngle() * Initialization.MultiplierForGyro) < Math.abs(angle)) {
			Robot.displaySensors();
			if (angle > 0) {
				Initialization.gearaffesDrive.arcadeDrive(0, power);
			} else {
				Initialization.gearaffesDrive.arcadeDrive(0, -power);
			}
			return false;
		} else {
			Initialization.gearaffesDrive.arcadeDrive(0, 0);
			return true;
		}
	}
	
	public static void elevatorToBase(double speed) {
		int successes = 0;
		while (successes < 5) {
			Initialization.elevator.set(-speed); // assuming negative speed moves the elevator down
			if (Math.abs(Initialization.elevatorEncoder.getRate()) <= 0.1) {
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
	
	public static void determineAutonomousSequence() {
		//Switch Sequence
		if(Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition ==1) {
			Initialization.switchSequence.add((Void)-> move(168 - Initialization.robotDepth/2,Initialization.autoMovePower) );
			Initialization.switchSequence.add((Void)-> rotate(90,Initialization.autoRotatePower) );
			Initialization.switchSequence.add((Void)-> moveUntilContact(55.56 - Initialization.robotWidth/2, Initialization.autoMoveUntilContactPower, 0.2));
		} else if(Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition ==2) {
			Initialization.switchSequence.add((Void)-> move(59-Initialization.robotDepth, Initialization.autoMovePower) );
			Initialization.switchSequence.add((Void)-> rotate(-90, Initialization.autoRotatePower) );
			Initialization.switchSequence.add((Void)-> move(42+Initialization.robotWidth/2, Initialization.autoMovePower));				
			Initialization.switchSequence.add((Void)-> rotate(90, Initialization.autoRotatePower));
			Initialization.switchSequence.add((Void)-> moveUntilContact(81, Initialization.autoMoveUntilContactPower, 0.2));	
		} else if(Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition ==3) {
			Initialization.switchSequence.add((Void)-> move(228-Initialization.robotWidth/2,Initialization.autoMovePower));					
			Initialization.switchSequence.add((Void)-> rotate(-90, Initialization.autoRotatePower) );
			Initialization.switchSequence.add((Void)-> move(264-Initialization.robotWidth, Initialization.autoMovePower));
			Initialization.switchSequence.add((Void)-> rotate(-90, Initialization.autoRotatePower));
			Initialization.switchSequence.add((Void)-> move(60,Initialization.autoMovePower));
			Initialization.switchSequence.add((Void)-> rotate(-90, Initialization.autoRotatePower));
			Initialization.switchSequence.add((Void)-> moveUntilContact(55.56 - Initialization.robotWidth/2, Initialization.autoMoveUntilContactPower, 0.2));	
		} else if(Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition ==1) {
			Initialization.switchSequence.add((Void)-> move(228-Initialization.robotWidth/2,Initialization.autoMovePower));
			Initialization.switchSequence.add((Void)-> rotate(90, Initialization.autoRotatePower));
			Initialization.switchSequence.add((Void)-> move(264-Initialization.robotWidth, Initialization.autoMovePower));
			Initialization.switchSequence.add((Void)-> rotate(90, Initialization.autoRotatePower));
			Initialization.switchSequence.add((Void)-> move(60,Initialization.autoMovePower));
			Initialization.switchSequence.add((Void)-> rotate(90, Initialization.autoRotatePower));
			Initialization.switchSequence.add((Void)-> moveUntilContact(55.56 - Initialization.robotWidth/2, Initialization.autoMoveUntilContactPower, 0.2));	
		} else if(Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition ==2) {
			Initialization.switchSequence.add((Void)-> move(59-Initialization.robotDepth, Initialization.autoMovePower));
			Initialization.switchSequence.add((Void)-> rotate(90, Initialization.autoRotatePower));
			Initialization.switchSequence.add((Void)-> move(66-Initialization.robotWidth/2, Initialization.autoMovePower));
			Initialization.switchSequence.add((Void)-> rotate(-90, Initialization.autoRotatePower));
			Initialization.switchSequence.add((Void)-> moveUntilContact(81, Initialization.autoMoveUntilContactPower, 0.2));		
		} else if(Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition ==3) {
			Initialization.switchSequence.add((Void)-> move(168 - Initialization.robotDepth/2, Initialization.autoMovePower));
			Initialization.switchSequence.add((Void)-> rotate(-90,Initialization.autoRotatePower));
			Initialization.switchSequence.add((Void)-> moveUntilContact(55.56 - Initialization.robotWidth/2, Initialization.autoMoveUntilContactPower, 0.2));
		}			
		
		//Scale Sequence
		if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition == 1) {
			Initialization.scaleSequence.add((Void)-> move(323.65 - Initialization.robotDepth/2,0.45) );
			Initialization.scaleSequence.add((Void)-> rotate(90,0.45) );			
		} else if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition == 2) {
			Initialization.scaleSequence.add((Void)-> move(59-Initialization.robotDepth, 0.45) );
			Initialization.scaleSequence.add((Void)-> rotate(-90, 0.45) );
			Initialization.scaleSequence.add((Void)-> move(120+Initialization.robotWidth/2, 0.45));
			Initialization.scaleSequence.add((Void)-> rotate(90, 0.45));
			Initialization.scaleSequence.add((Void)-> move(264.65, 0.45));
			Initialization.scaleSequence.add((Void)-> rotate(90, 0.45));
		} else if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition == 3) {
			Initialization.scaleSequence.add((Void)-> move(228-Initialization.robotWidth/2,0.45));
			Initialization.scaleSequence.add((Void)-> rotate(-90, 0.45) );
			Initialization.scaleSequence.add((Void)-> move(264-Initialization.robotWidth, 0.45));
			Initialization.scaleSequence.add((Void)-> rotate(90, 0.45));
			Initialization.scaleSequence.add((Void)-> move(95.65,0.45));
			Initialization.scaleSequence.add((Void)-> rotate(90, 0.45));
		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition == 1) {
			Initialization.scaleSequence.add((Void)-> move(228-Initialization.robotWidth/2,0.45));
			Initialization.scaleSequence.add((Void)-> rotate(90, 0.45) );
			Initialization.scaleSequence.add((Void)-> move(264-Initialization.robotWidth, 0.45));
			Initialization.scaleSequence.add((Void)-> rotate(-90, 0.45));
			Initialization.scaleSequence.add((Void)-> move(95.65,0.45));
			Initialization.scaleSequence.add((Void)-> rotate(-90, 0.45));
		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition == 2) {
			Initialization.scaleSequence.add((Void)-> move(59-Initialization.robotDepth, 0.45));
			Initialization.scaleSequence.add((Void)-> rotate(90, 0.45));
			Initialization.scaleSequence.add((Void)-> move(120-Initialization.robotWidth/2, 0.45));
			Initialization.scaleSequence.add((Void)-> rotate(-90, 0.45));
			Initialization.scaleSequence.add((Void)-> move(264.65, 0.45));
		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition == 3) {
			Initialization.scaleSequence.add((Void)-> move(323.65 - Initialization.robotDepth/2, 0.45));
			Initialization.scaleSequence.add((Void)-> rotate(-90,0.45));
		}
	}
}
