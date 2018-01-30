/***************************************************************************************
*    Title: Gearaffes Robotics Team FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {

	// Autonomous Routines
	
	public static boolean crossAutoline() {
		return move(126, 0.7);
	}
	
	public static void placeCubeOnSwitch() {
		Timer.delay(Initialization.autoDelayTime);
		if(Robot.autoProcess < Initialization.switchSequence.size()) {
			if(Initialization.switchSequence.get(Robot.autoProcess).apply(null)) {
				Robot.resetSensors();
				Robot.autoProcess++;
				Autonomous.successesContact = 0;
				Initialization.gearaffesPID.reset();
				Initialization.gearaffesPID.enable();
			}
		}
		SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance())/12));
	}
	
	public static boolean autoElevatorHeight(double height) {
		boolean goSlowBottom = (Math.abs(Initialization.elevatorEncoder.getDistance()) < 12 && Math.signum(Initialization.elevator.get()) == -1);
		boolean goSlowTop = (Math.abs(Initialization.elevatorEncoder.getDistance()) > 72 && Math.signum(Initialization.elevator.get()) == 1);
		double speed = goSlowBottom ? Initialization.automationBottomSpeed : goSlowTop ? Initialization.automationTopSpeed : Initialization.automationHighSpeed;
		return setElevatorHeight(height, speed);
	}
	
	public static void placeCubeOnScale() {
		Timer.delay(Initialization.autoDelayTime);
		if(Robot.autoProcess < Initialization.scaleSequence.size()) {
			if(Initialization.scaleSequence.get(Robot.autoProcess).apply(null)) {
				Robot.resetSensors();
				Robot.autoProcess++;
				successesContact = 0;
				Initialization.gearaffesPID.reset();
				Initialization.gearaffesPID.enable();
			}
		}
		SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance())/12));
	}
	
	//building blocks
	
	public static boolean move(double dist, double speed) {
		if (Math.abs(Initialization.leftDriveEncoder.getDistance()) < Math.abs(dist)) {
			if (dist > 0) {
				Initialization.gearaffesDrive.arcadeDrive(speed, Initialization.gearaffesPID.get());//-Initialization.gyro.getAngle() * Initialization.move_KP);
			} else {
				Initialization.gearaffesDrive.arcadeDrive(-speed, Initialization.gearaffesPID.get());//-Initialization.gyro.getAngle() * Initialization.move_KP);
			}
			Robot.displaySensors();
			return false;
		} else {
		 Initialization.gearaffesDrive.arcadeDrive(0, 0);
		 return true;
		}
	}
	
	public static int successesContact = 0;
	public static boolean moveUntilContact(double dist, double highSpeed, double lowSpeed) {
		Robot.displaySensors();
		if(successesContact<5) {
			double speed = Math.abs(Initialization.leftDriveEncoder.getDistance()) < 0.8*dist ? highSpeed : lowSpeed;
			Initialization.gearaffesDrive.arcadeDrive(speed, -Initialization.gyro.getAngle() * Initialization.move_KP);
			if(Math.abs(Initialization.leftDriveEncoder.getRate())<=0.5){
				successesContact++;
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
	
	public static boolean elevatorToBase(double speed) {
		if(!Initialization.bottomLimitSwitch.get()) {
			Initialization.elevator.set(0);
			Teleop.automationInProgress = false;
			Teleop.startElevatorRumble(0.5, false, true);
			return true;
		} else {
			Initialization.elevator.set(-speed);
			Teleop.automationInProgress = true;
			return false;
		}
	}

	public static boolean setElevatorHeight(double height, double speed) {	
		double dist = height - Math.abs(Initialization.elevatorEncoder.getDistance());
		if (height==0) {
			return elevatorToBase(speed);
		} else if (dist > 0) { 
			if (Math.abs(Initialization.elevatorEncoder.getDistance()) < height - 1) { 
				Initialization.elevator.set(speed);
				return false;
			} else {
				Initialization.elevator.set(0);
				Teleop.automationInProgress = false;
				Teleop.startElevatorRumble(0.5, false, true);
				return true;
			}
		} else if (dist < 0) {
			if (Math.abs(Initialization.elevatorEncoder.getDistance()) > height + 1) { 
				Initialization.elevator.set(-speed);
				return false;
			} else {
				Initialization.elevator.set(0);
				Teleop.automationInProgress = false;
				Teleop.startElevatorRumble(0.5, false, true);
				return true;
			}
		} else {
			Teleop.automationInProgress = false;
			Teleop.startElevatorRumble(0.5, false, true);
			return true;
		}
	}
	
	public static void getMatchData() {
		 try {
			 Initialization.ourSwitchPosition = DriverStation.getInstance().getGameSpecificMessage().charAt(0);
			 Initialization.scalePosition = DriverStation.getInstance().getGameSpecificMessage().charAt(1);
			 Initialization.opposingSwitchPosition = DriverStation.getInstance().getGameSpecificMessage().charAt(2);
		 } catch(NullPointerException e) {
			 System.err.println("One or more of the field element positions could not be determined");
		 }
	}
	
	public static boolean delay(double delay) {
		Timer.delay(delay);
		return true;
	}
	
	public static void determineAutonomousSequence() {
		//Switch Sequence
		Initialization.switchSequence.clear();
		
		if(Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition ==1) {
			Initialization.switchSequence.add((Void)-> move(168 - Initialization.robotDepth/2,Initialization.autoMoveSpeed) & autoElevatorHeight(20));
			Initialization.switchSequence.add((Void)-> rotate(90,Initialization.autoRotateSpeed) );
			Initialization.switchSequence.add((Void)-> moveUntilContact(55.56 - Initialization.robotWidth/2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
		
		} else if(Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition ==2) {
			Initialization.switchSequence.add((Void)-> move(59-Initialization.robotDepth, Initialization.autoMoveSpeed) & autoElevatorHeight(20) );
			Initialization.switchSequence.add((Void)-> rotate(-90, Initialization.autoRotateSpeed) );
			Initialization.switchSequence.add((Void)-> move(95+Initialization.robotWidth/2, Initialization.autoMoveSpeed));				
			Initialization.switchSequence.add((Void)-> rotate(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void)-> moveUntilContact(81, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));	
		
		} else if(Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition ==3) {
			Initialization.switchSequence.add((Void)-> move(228-Initialization.robotWidth/2,Initialization.autoMoveSpeed)& autoElevatorHeight(20) );					
			Initialization.switchSequence.add((Void)-> rotate(-90, Initialization.autoRotateSpeed) );
			Initialization.switchSequence.add((Void)-> move(264-Initialization.robotWidth, Initialization.autoMoveSpeed)); // shaved off 24"
			Initialization.switchSequence.add((Void)-> rotate(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void)-> move(60,Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void)-> rotate(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void)-> moveUntilContact(55.56 - Initialization.robotWidth/2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));	//here too
		
		} else if(Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition ==1) {
			Initialization.switchSequence.add((Void)-> move(228-Initialization.robotWidth/2,Initialization.autoMoveSpeed)& autoElevatorHeight(20) );
			Initialization.switchSequence.add((Void)-> rotate(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void)-> move(240-Initialization.robotWidth, Initialization.autoMoveSpeed)); // shaved off 24" here too
			Initialization.switchSequence.add((Void)-> rotate(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void)-> move(60,Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void)-> rotate(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void)-> moveUntilContact(31.56 - Initialization.robotWidth/2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));	
		
		} else if(Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition ==2) {
			Initialization.switchSequence.add((Void)-> moveUntilContact(140-Initialization.robotDepth, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow)& autoElevatorHeight(20) );		
		
		} else if(Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition ==3) {
			Initialization.switchSequence.add((Void)-> move(168 - Initialization.robotDepth/2, Initialization.autoMoveSpeed)& autoElevatorHeight(20) );
			Initialization.switchSequence.add((Void)-> rotate(-90,Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void)-> moveUntilContact(55.56 - Initialization.robotWidth/2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
		}			
		
		//Scale Sequence
		Initialization.scaleSequence.clear();
		
		if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition == 1) {
			Initialization.scaleSequence.add((Void)-> move(313.65 - Initialization.robotDepth/2, Initialization.autoMoveSpeed) & autoElevatorHeight(72) );
			Initialization.scaleSequence.add((Void)-> rotate(90, Initialization.autoRotateSpeed) );
			Initialization.scaleSequence.add((Void)-> move(4, Initialization.autoMoveSpeed) );
		
		} else if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition == 2) {
			Initialization.scaleSequence.add((Void)-> move(59-Initialization.robotDepth, Initialization.autoMoveSpeed) & autoElevatorHeight(72));
			Initialization.scaleSequence.add((Void)-> rotate(-90, Initialization.autoRotateSpeed) );
			Initialization.scaleSequence.add((Void)-> move(173+Initialization.robotWidth/2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void)-> rotate(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void)-> move(264.65, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void)-> rotate(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void)-> move(4,Initialization.autoMoveSpeed) );
		
		} else if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition == 3) {
			Initialization.scaleSequence.add((Void)-> move(228-Initialization.robotWidth/2, Initialization.autoMoveSpeed) & autoElevatorHeight(72));
			Initialization.scaleSequence.add((Void)-> rotate(-90, Initialization.autoRotateSpeed) );
			Initialization.scaleSequence.add((Void)-> move(276-Initialization.robotWidth, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void)-> rotate(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void)-> move(95.65,Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void)-> rotate(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void)-> move(16, Initialization.autoMoveSpeed) );
		
		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition == 1) {
			Initialization.scaleSequence.add((Void)-> move(227-Initialization.robotWidth/2, Initialization.autoMoveSpeed) & autoElevatorHeight(72));
			Initialization.scaleSequence.add((Void)-> rotate(90, Initialization.autoRotateSpeed) );
			Initialization.scaleSequence.add((Void)-> move(252-Initialization.robotWidth, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void)-> rotate(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void)-> move(83.65, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void)-> rotate(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void)-> move(22, Initialization.autoMoveSpeed) );
		
		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition == 2) {
			Initialization.scaleSequence.add((Void)-> move(59-Initialization.robotDepth, Initialization.autoMoveSpeed) & autoElevatorHeight(72));
			Initialization.scaleSequence.add((Void)-> rotate(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void)-> move(67-Initialization.robotWidth/2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void)-> rotate(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void)-> move(268.65, Initialization.autoMoveSpeed));
		
		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition == 3) {
			Initialization.scaleSequence.add((Void)-> move(323.65 - Initialization.robotDepth/2, Initialization.autoMoveSpeed) & autoElevatorHeight(72));
			Initialization.scaleSequence.add((Void)-> rotate(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void)-> move(4, Initialization.autoMoveSpeed) );
		}
	}
}