/***************************************************************************************
*    Title: Gearaffes Robotics Team FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
 
	public static boolean crossAutoline() {
		return DriveBase.moveAndBrake(126, 0.7);
	}

	public static boolean placeCubeOnSwitch() {	
			if (Robot.autoProcess < Initialization.switchSequence.size()) {
				if (Initialization.switchSequence.get(Robot.autoProcess).apply(null)) {
					BuildingBlocks.resetSomeSensors();
					Robot.autoProcess++;
					DriveBase.successesContact = 0;
					Initialization.gearaffesPID.reset();
					Initialization.gearaffesPID.enable();
					DriveBase.setBraking(false);
				}
				SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
				return false;
			} else {
				Teleop.switchAutomationInProgress = false;
				Teleop.scaleAutomationInProgress = false;
				SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
				return true;
			}
	}


	public static boolean placeCubeOnScale() {
		if (Robot.autoProcess < Initialization.scaleSequence.size()) {
			if (Initialization.scaleSequence.get(Robot.autoProcess).apply(null)) {
				BuildingBlocks.resetSomeSensors();
				Robot.autoProcess++;
				DriveBase.successesContact = 0;
				Initialization.gearaffesPID.reset();
				Initialization.gearaffesPID.enable();
			}
			SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
			return false;
		} else {
			Teleop.switchAutomationInProgress = false;
			Teleop.scaleAutomationInProgress = false;
			SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
			return true;
		}
	}
	
	public static boolean placeCubeOnScaleThenSwitch() {
		if (Robot.autoProcess < Initialization.scaleThenSwitchSequence.size()) {
			if (Initialization.scaleThenSwitchSequence.get(Robot.autoProcess).apply(null)) {
				BuildingBlocks.resetSomeSensors();
				Robot.autoProcess++;
				DriveBase.successesContact = 0;
				Initialization.gearaffesPID.reset();
				Initialization.gearaffesPID.enable();
			}
			SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
			return false;
		} else {
			Teleop.switchAutomationInProgress = false;
			Teleop.scaleAutomationInProgress = false;
			SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
			return true;
		}
		
	}
	
	public static boolean placeCubeOnSwitchThenSwitch() {
		if (Robot.autoProcess < Initialization.switchThenSwitchSequence.size()) {
			if (Initialization.switchThenSwitchSequence.get(Robot.autoProcess).apply(null)) {
				BuildingBlocks.resetSomeSensors();
				Robot.autoProcess++;
				DriveBase.successesContact = 0;
				Initialization.gearaffesPID.reset();
				Initialization.gearaffesPID.enable();
			}
			SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
			return false;
		} else {
			Teleop.switchAutomationInProgress = false;
			Teleop.scaleAutomationInProgress = false;
			SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
			return true;
		}
	}
	
	public static void determineAutonomousSequence() {
		// Switch Sequence
		Initialization.switchSequence.clear();

		// Ipsilateral Switch Station 1
		if (Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition.equals("1")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(168 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveUntilContact(55.56 - Initialization.robotWidth / 2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.switchSequence.add((Void) -> DriveBase.moveUntilContact(12, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> Elevator.eject());
			Initialization.switchSequence.add((Void) -> Elevator.autoElevatorHeight(6));
			Initialization.switchSequence.add((Void) -> Elevator.eject());
			
		// Contralateral Switch Station 1
		} else if (Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition.equals("1")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(244 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(264 - Initialization.robotWidth, Initialization.autoMoveSpeed)); 																																									// too				Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(64, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveUntilContact(55.56 - Initialization.robotWidth / 2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.switchSequence.add((Void) -> DriveBase.moveUntilContact(12, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> Elevator.eject());
			Initialization.switchSequence.add((Void) -> Elevator.autoElevatorHeight(6));
			Initialization.switchSequence.add((Void) -> Elevator.eject());
			
		// Ipsilateral Switch Station 2
		} else if (Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition.equals("2")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.switchSequence.add((Void) -> DriveBase.moveUntilContact(140 - Initialization.robotDepth, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.switchSequence.add((Void) -> DriveBase.moveUntilContact(12, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> Elevator.eject());
			Initialization.switchSequence.add((Void) -> Elevator.autoElevatorHeight(6));
			Initialization.switchSequence.add((Void) -> Elevator.eject());
			// Untested Portion - Moves Robot to Exchange Zone
			/*Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(-70, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(60 - Initialization.robotDepth, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(34, Initialization.autoMoveSpeed));*/
				
		// Contralateral Switch Station 2
		} else if (Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition.equals("2")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(59 - Initialization.robotDepth, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(107 + Initialization.robotWidth / 2, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveUntilContact(81 - Initialization.robotDepth, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.switchSequence.add((Void) -> DriveBase.moveUntilContact(12, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> Elevator.eject());
			Initialization.switchSequence.add((Void) -> Elevator.autoElevatorHeight(6));
			Initialization.switchSequence.add((Void) -> Elevator.eject());
			// Untested Portion - Moves Robot to Exchange Zone
			/*Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(-70, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(60 - Initialization.robotDepth, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(34, Initialization.autoMoveSpeed));*/
					
		// Ipsilateral Switch Station 3
		} else if (Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(168 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveUntilContact(55.56 - Initialization.robotWidth / 2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.switchSequence.add((Void) -> DriveBase.moveUntilContact(12, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> Elevator.eject());
			Initialization.switchSequence.add((Void) -> Elevator.autoElevatorHeight(6));
			Initialization.switchSequence.add((Void) -> Elevator.eject());
			
		// Contralateral Switch Station 3
		} else if (Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(244 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake((246- Initialization.robotWidth)/2-31, Initialization.autoMoveSpeed)); 
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(62, Initialization.bumpSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake((246 - Initialization.robotWidth)/2-31, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(64, Initialization.autoMoveSpeed)); 
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveUntilContact(55.56 - Initialization.robotWidth / 2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow)); 
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.switchSequence.add((Void) -> DriveBase.moveUntilContact(12, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> Elevator.eject());
			Initialization.switchSequence.add((Void) -> Elevator.autoElevatorHeight(6));
			Initialization.switchSequence.add((Void) -> Elevator.eject());	
		} 

		// Scale Sequence
		Initialization.scaleSequence.clear();

		// Ipsilateral Scale Station 1
		if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition.equals("1")) {
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(325.65 - 24 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.rotateAndBrake(45, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(Initialization.autoScaleHeight));
			Initialization.scaleSequence.add((Void) -> DriveBase.move(11, Initialization.autoMoveContactLow));
			Initialization.scaleSequence.add((Void) -> Elevator.eject());
			Initialization.scaleSequence.add((Void) -> DriveBase.move(-8, Initialization.autoMoveContactLow));
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(0));
			
		// Contralateral Scale Station 1
		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition.equals("1")) {
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(242.375 + 2 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(264 - Initialization.robotWidth, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(83.65, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(Initialization.autoScaleHeight));
			Initialization.scaleSequence.add((Void) -> DriveBase.move(4, Initialization.autoMoveContactLow) & Elevator.endeffOut());
			//Initialization.scaleSequence.add((Void) -> Elevator.eject());
			//Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(1));
			Initialization.scaleSequence.add((Void) -> DriveBase.move(-4, Initialization.autoMoveContactLow) & Elevator.endeffIn());
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(12));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(0));

		// Ipsilateral Scale Station 2
		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition.equals("2")) {
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(59 - Initialization.robotDepth, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(67 - Initialization.robotWidth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(4, Initialization.autoMoveContactLow) & Elevator.eject());
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(-4, Initialization.autoMoveContactLow));
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(12));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(0));
			
		// Contralateral Scale Station 2
		} else if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition.equals("2")) {
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(59 - Initialization.robotDepth, Initialization.autoMoveSpeed) & Elevator.autoElevatorHeight(62));
			Initialization.scaleSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(173 + Initialization.robotWidth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(264.65, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(4, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(4, Initialization.autoMoveContactLow) & Elevator.eject());
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(-4, Initialization.autoMoveContactLow));
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(12));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(0));

		// Ipsilateral Scale Station 3
		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(325.65 - 24 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.rotateAndBrake(-45, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(Initialization.autoScaleHeight));
			Initialization.scaleSequence.add((Void) -> DriveBase.move(11, Initialization.autoMoveContactLow));
			Initialization.scaleSequence.add((Void) -> Elevator.eject());
			Initialization.scaleSequence.add((Void) -> DriveBase.move(-8, Initialization.autoMoveContactLow));
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(12));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(0));
		
		// Contralateral Scale Station 3
		} else if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(244 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(276 + 8 - Initialization.robotWidth, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(92.65, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(Initialization.autoScaleHeight));
			Initialization.scaleSequence.add((Void) -> DriveBase.move(4, Initialization.autoMoveContactLow) & Elevator.endeffOut());
			//Initialization.scaleSequence.add((Void) -> Elevator.eject());
			//Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(1));
			Initialization.scaleSequence.add((Void) -> DriveBase.move(-4, Initialization.autoMoveContactLow) & Elevator.endeffIn());
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(12));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(0));
		}

		// Scale Then Switch Sequence
		Initialization.scaleThenSwitchSequence.clear();

		if (Initialization.scalePosition == 'R' && Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveAndBrake(325.65 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed) & Elevator.autoElevatorHeight(62));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			//Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(Initialization.autoScaleHeight));
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(4, Initialization.autoMoveContactLow) & Elevator.eject());
			Initialization.scaleSequence.add((Void) -> DriveBase.moveAndBrake(-4, Initialization.autoMoveContactLow));
			//Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(0));
			// Untested Portion Below 
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveAndBrake(84.915, Initialization.autoMoveSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveAndBrake(91.75 - (33.69 + Initialization.robotDepth / 2), Initialization.autoMoveSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveUntilContact(32.735, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			//Initialization.scaleThenSwitchSequence.add((Void) -> Grabber.cubeToEndEffector());
			Initialization.scaleThenSwitchSequence.add((Void) -> Elevator.eject());
			
		} else if (Initialization.scalePosition == 'L' && Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition.equals("1")) {
			Initialization.scaleThenSwitchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveAndBrake(325.65 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			//Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(Initialization.autoScaleHeight));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveAndBrake(4, Initialization.autoMoveContactLow));
			Initialization.scaleThenSwitchSequence.add((Void) -> Elevator.eject());
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveAndBrake(-4, Initialization.autoMoveContactLow));
			//Initialization.scaleSequence.add((Void) -> Elevator.autoElevatorHeight(0));
			// Untested Portion Below
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveAndBrake(78.915, Initialization.autoMoveSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveAndBrake(92.75 - (33.69 + Initialization.robotDepth / 2), Initialization.autoMoveSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveUntilContact(32.735, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			//Initialization.scaleThenSwitchSequence.add((Void) -> Grabber.cubeToEndEffector());
			Initialization.scaleThenSwitchSequence.add((Void) -> Elevator.eject());
		}
		
		// Switch Then Switch Sequence
			Initialization.switchThenSwitchSequence.clear();
			/*
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(112, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveUntilContact(28 - Initialization.robotDepth, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> Elevator.eject());
			// Untested Portion Below
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveAndBrake(-54, Initialization.autoMoveSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveAndBrake(54, Initialization.autoMoveSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveUntilContact(22, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
				// Align Cube here
			Initialization.scaleThenSwitchSequence.add((Void) -> Grabber.cubeToEndEffector());
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveAndBrake(-10, Initialization.autoMoveSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveAndBrake(54, Initialization.autoMoveSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveAndBrake(43.2, Initialization.autoMoveSpeed));
			Initialization.scaleThenSwitchSequence.add((Void) -> DriveBase.moveUntilContact(10.8, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			// Additional Untested Portion - Moves Robot to Exchange Zone
			/*Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(-70, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(60 - Initialization.robotDepth, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> DriveBase.moveAndBrake(34, Initialization.autoMoveSpeed));*/	
	}
}
