/***************************************************************************************
*    Title: Gearaffes Robotics Team FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {

	// Autonomous Routines

	public static boolean crossAutoline() {
		return BuildingBlocks.moveAndBrake(126, 0.7);
	}

	public static void placeCubeOnSwitch() {
		Timer.delay(Initialization.autoDelayTime);
		if (Robot.autoProcess < Initialization.switchSequence.size()) {
			if (Initialization.switchSequence.get(Robot.autoProcess).apply(null)) {
				BuildingBlocks.resetSomeSensors();
				Robot.autoProcess++;
				BuildingBlocks.successesContact = 0;
				Initialization.gearaffesPID.reset();
				Initialization.gearaffesPID.enable();
			}
		} else {
			Teleop.switchAutomationInProgress = false;
			Teleop.scaleAutomationInProgress = false;
		}
		SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
	}

	public static void placeCubeOnScale() {
		Timer.delay(Initialization.autoDelayTime);
		if (Robot.autoProcess < Initialization.scaleSequence.size()) {
			if (Initialization.scaleSequence.get(Robot.autoProcess).apply(null)) {
				BuildingBlocks.resetSomeSensors();
				Robot.autoProcess++;
				BuildingBlocks.successesContact = 0;
				Initialization.gearaffesPID.reset();
				Initialization.gearaffesPID.enable();
			}
		} else {
			Teleop.switchAutomationInProgress = false;
			Teleop.scaleAutomationInProgress = false;
		}
		SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
	}

	public static void twoCubeAutoFront() {
		Timer.delay(Initialization.autoDelayTime);
		if (Robot.autoProcess < Initialization.twoCubeSequence.size()) {
			if (Initialization.twoCubeSequence.get(Robot.autoProcess).apply(null)) {
				BuildingBlocks.resetSomeSensors();
				Robot.autoProcess++;
				BuildingBlocks.successesContact = 0;
				Initialization.gearaffesPID.reset();
				Initialization.gearaffesPID.enable();
			}
		}
		SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
	}
	
	public static void sideScale() {
		Timer.delay(Initialization.autoDelayTime);
		if (Robot.autoProcess < Initialization.sideScaleSequence.size()) {
			if (Initialization.sideScaleSequence.get(Robot.autoProcess).apply(null)) {
				BuildingBlocks.resetSomeSensors();
				Robot.autoProcess++;
				BuildingBlocks.successesContact = 0;
				Initialization.gearaffesPID.reset();
				Initialization.gearaffesPID.enable();
			}
		}
		SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
	}

	public static void determineAutonomousSequence() {
		// Switch Sequence
		Initialization.switchSequence.clear();

		if (Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition.equals("1")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(168 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(55.56 - Initialization.robotWidth / 2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			//Initialization.switchSequence.add((Void) -> BuildingBlocks.eject());
		} else if (Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition.equals("2")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(59 - Initialization.robotDepth, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(95 + Initialization.robotWidth / 2, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(64.8, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(16.2 - Initialization.robotDepth, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));

		} else if (Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(238 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(270 - Initialization.robotWidth, Initialization.autoMoveSpeed)); // shaved off																							// 24"
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(60, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(55.56 - Initialization.robotWidth / 2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow)); // here too

		} else if (Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition.equals("1")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(238 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(270 - Initialization.robotWidth, Initialization.autoMoveSpeed)); 																																									// too
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(60, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(55.56 - Initialization.robotWidth / 2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));

		} else if (Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition.equals("2")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(112, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(28 - Initialization.robotDepth, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));

		} else if (Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(168 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(55.56 - Initialization.robotWidth / 2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
		}

		// Scale Sequence
		Initialization.scaleSequence.clear();

		if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition.equals("1")) {
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(325.65 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			//Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(1));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(72));
			//Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(1));// replace with end effector deposition code

		} else if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition.equals("2")) {
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(59 - Initialization.robotDepth, Initialization.autoMoveSpeed) & BuildingBlocks.autoElevatorHeight(72));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(173 + Initialization.robotWidth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(264.65, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(4, Initialization.autoMoveSpeed));

		} else if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(242.375 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(276 - Initialization.robotWidth, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(83.65, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed) & BuildingBlocks.autoElevatorHeight(72));
			//Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(16, Initialization.autoMoveSpeed));
			//Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(72));

		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition.equals("1")) {
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(242.375 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(276 - Initialization.robotWidth, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(83.65, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed) & BuildingBlocks.autoElevatorHeight(72));
			//Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(22, Initialization.autoMoveSpeed));

		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition.equals("2")) {
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(59 - Initialization.robotDepth, Initialization.autoMoveSpeed) & BuildingBlocks.autoElevatorHeight(72));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(67 - Initialization.robotWidth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(268.65, Initialization.autoMoveSpeed));

		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(323.65 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed) & BuildingBlocks.autoElevatorHeight(72));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(1));// replace with end effector deposition code
		}

		// Two Cube Sequence
		Initialization.twoCubeSequence.clear();

		if (Initialization.scalePosition == 'R' && Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveAndBrake(323.65 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed) & BuildingBlocks.autoElevatorHeight(72));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.delay(1));// replace with end effector deposition code
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveAndBrake(94.915, Initialization.autoMoveSpeed) & BuildingBlocks.autoElevatorHeight(30));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveAndBrake(91.75 - (33.69 + Initialization.robotDepth / 2), Initialization.autoMoveSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveUntilContact(32.735, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));

		} else if (Initialization.scalePosition == 'L' && Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition.equals("1")) {
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveAndBrake(325.65 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed) & BuildingBlocks.autoElevatorHeight(72));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.delay(1));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.delay(1));// replace with end effector deposition code
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveAndBrake(84.915, Initialization.autoMoveSpeed) & BuildingBlocks.autoElevatorHeight(30));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotate(-90, Initialization.autoRotateSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveAndBrake(91.75 - (33.69 + Initialization.robotDepth / 2), Initialization.autoMoveSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotate(90, Initialization.autoRotateSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveUntilContact(32.735, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
		}
		
		// Side Scale Sequence
		Initialization.sideScaleSequence.clear();
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(196-Initialization.robotDepth, 0.7));
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.bankingRotate(Math.toDegrees(Math.atan((56.5-Initialization.robotWidth/2)/80)), 0.35));
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(Math.sqrt(Math.pow(56.5-Initialization.robotWidth/2, 2) + Math.pow(80, 2)), 0.45));
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.bankingRotate(-Math.toDegrees(Math.atan((56.5-Initialization.robotWidth/2)/80)), 0.35));
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(72));
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.delay(3));
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(0));
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(-12, 0.45));
	}
}
