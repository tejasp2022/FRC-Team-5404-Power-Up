/***************************************************************************************
*    Title: Gearaffes Robotics Team FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {


	public static ArrayList<Double> timeList = new ArrayList<Double>();
	public static ArrayList<Double> BRList = new ArrayList<Double>();
	public static ArrayList<Double> FRList = new ArrayList<Double>();
	public static ArrayList<Double> BLList = new ArrayList<Double>();
	public static ArrayList<Double> FLList = new ArrayList<Double>();
	public static ArrayList<Double> elvList = new ArrayList<Double>();
	public static ArrayList<Boolean> ejectPistonList = new ArrayList<Boolean>();
	public static ArrayList<Boolean> grabberPistonList = new ArrayList<Boolean>();
	public static ArrayList<Double> gyroList = new ArrayList<Double>();
	public static ArrayList<Double> encoderList = new ArrayList<Double>();
	public static ArrayList<Double> grabberList = new ArrayList<Double>();
	
	public static void record() {
		timeList.add(Timer.getFPGATimestamp());
		BRList.add(Initialization.BR.get());
		FRList.add(Initialization.FR.get());
		BLList.add(Initialization.BL.get());
		FLList.add(Initialization.FL.get());
		elvList.add(Initialization.elevator.get());
		grabberList.add(Initialization.grabberMotorController.get());
		ejectPistonList.add(Initialization.endEffectorPiston.get());
		grabberPistonList.add(Initialization.grabberPiston.get());
		gyroList.add(Initialization.gyro.getAngle());
		encoderList.add(Initialization.rightDriveEncoder.getDistance());
	}
	
	public static void printer() {
		System.out.println("FRList");
		for(double i: FRList) {
			System.out.println("FRList.add(" + i + ");");
		}
		
		System.out.println("\nBRList");
		for(double i: BRList) {
			System.out.println("BRList.add(" + i + ");");
		}
		
		System.out.println("\nFLList");
		for(double i: FLList) {
			System.out.println("FLList.add(" + i + ");");
		}
		
		System.out.println("\nBLList");
		for(double i: BLList) {
			System.out.println("BLList.add(" + i + ");");
		}
		
		System.out.println("\nelvList");
		for(double i: elvList) {
			System.out.println("elvList.add(" + i + ");");
		}
		
		System.out.println("\ngrabberList");
		for(double i: grabberList) {
			System.out.println("grabberList.add(" + i + ");");
		}
		
		System.out.println("\nejectPistonList");
		for(boolean i: ejectPistonList) {
			System.out.println("ejectPistonList.add(" + i + ");");
		}
		
		System.out.println("\ngrabberPistonList");
		for(boolean i: grabberPistonList) {
			System.out.println("grabberPistonList.add(" + i + ");");
		}
	}

	public static ArrayList<Double> differenceInGyroList = new ArrayList<Double>();
	public static ArrayList<Double> differenceInEncoderList = new ArrayList<Double>();
	
	public static void playback(int i) {
		//for(int i = 0; i<timeList.size(); i++) {
		if(i<timeList.size()) {
		Initialization.BR.set(BRList.get(i));
			Initialization.FR.set(FRList.get(i));
			Initialization.BL.set(BLList.get(i));
			Initialization.FL.set(FLList.get(i));
			Initialization.elevator.set(elvList.get(i));
			//Initialization.endEffector.set(endEffectorList.get(i)); 
			differenceInGyroList.add(Math.abs(Math.abs(gyroList.get(i)) - Math.abs(Initialization.gyro.getAngle()))); // Theoretically, the difference should be 0
			differenceInEncoderList.add(Math.abs(encoderList.get(i) - Initialization.rightDriveEncoder.getDistance()));
			SmartDashboard.putNumber("Gyro Difference", Math.abs(Math.abs(gyroList.get(i)) - Math.abs(Initialization.gyro.getAngle())));
			SmartDashboard.putNumber("Encoder Difference", Math.abs(encoderList.get(i) - Initialization.rightDriveEncoder.getDistance()));
		} else {
			SmartDashboard.putBoolean("Playback Robot", false);
		}
	}
	
	public static void playbackReverse(int i ) {
		if(i>=0) { 
			Initialization.BR.set(-BRList.get(i));
			Initialization.FR.set(-FRList.get(i));
			Initialization.BL.set(-BLList.get(i));
			Initialization.FL.set(-FLList.get(i));
			Initialization.elevator.set(-elvList.get(i));
			//Initialization.endEffector.set(endEffectorList.get(i)); 

		} else {
			SmartDashboard.putBoolean("Reverse Playback Robot", false);
		}
	}
	
	public static char currentAction = 'M'; 
	public static int startingIndex = 0;
 	public static ArrayList<String> stepList = new ArrayList<String>();
 	public static ArrayList<String> summaryList = new ArrayList<String>();
 	
	/*public static void sendActionSummary() {
		for (int i = 0; i<timeList.size(); i++) {
			if (FLList.get(i) == 0){
				stepList.add("Stationary at t=" + timeList.get(i) + " seconds");
			} else if(Math.signum(FLList.get(i)) == Math.signum(FRList.get(i))) { 
				stepList.add("Rotating at t=" + timeList.get(i) + " seconds");
			} else {
				stepList.add("Moving Forward at t=" + timeList.get(i) + " seconds");
			}
		}
		for(int i=0; i<stepList.size(); i++){
			if (stepList.get(i).charAt(0) != currentAction){
				if(currentAction == 'M'){
					double distance = encoderList.get(i-1) - encoderList.get(startingIndex);
					summaryList.add("Moved " +  distance + " inches");
				} else if (currentAction == 'R'){
					double angle = gyroList.get(i-1) - gyroList.get(startingIndex);
					summaryList.add("Rotated " +  angle + " degrees");
				}
				startingIndex = i;
				currentAction = stepList.get(i).charAt(0);
			}
		}
		SmartDashboard.putStringArray("Action Summary", summaryList.toArray(new String[summaryList.size()]));
	}*/
	
	
	
	// Autonomous Routines

	public static boolean crossAutoline() {
		return BuildingBlocks.moveAndBrake(126, 0.7);
	}

	public static void placeCubeOnSwitch() {	
			if (Robot.autoProcess < Initialization.switchSequence.size()) {
				if (Initialization.switchSequence.get(Robot.autoProcess).apply(null)) {
					BuildingBlocks.resetSomeSensors();
					Robot.autoProcess++;
					BuildingBlocks.successesContact = 0;
					Initialization.gearaffesPID.reset();
					Initialization.gearaffesPID.enable();
					BuildingBlocks.setBraking(false);
				}
			} else {
				Teleop.switchAutomationInProgress = false;
				Teleop.scaleAutomationInProgress = false;
			}
		SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
	}


	public static void placeCubeOnScale() {
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
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(168 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(55.56 - Initialization.robotWidth / 2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.eject());
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(-(84 - Initialization.robotDepth)/2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
		
		} else if (Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition.equals("2")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(59 - Initialization.robotDepth, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(113 + Initialization.robotWidth / 2, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(64.8, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(16.2 - Initialization.robotDepth, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.eject());
			//Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(-(84 - Initialization.robotDepth)/2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
		
		} else if (Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(238 - Initialization.robotDepth / 3, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(282 - Initialization.robotWidth, Initialization.autoMoveSpeed)); 
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(64, Initialization.autoMoveSpeed)); 
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(55.56 - Initialization.robotWidth / 2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow)); 
			Initialization.switchSequence.add((Void) -> BuildingBlocks.eject());
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(-(84 - Initialization.robotDepth)/2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
	
		} else if (Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition.equals("1")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(238 - Initialization.robotDepth / 3, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(264 - Initialization.robotWidth, Initialization.autoMoveSpeed)); 																																									// too
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(76, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(55.56 - Initialization.robotWidth / 2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.eject());
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(-(84 - Initialization.robotDepth)/2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
		
		} else if (Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition.equals("2")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(112, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(28 - Initialization.robotDepth, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.eject());
			//Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(-(84 - Initialization.robotDepth)/2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			
		} else if (Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveAndBrake(168 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(55.56 - Initialization.robotWidth / 2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.switchSequence.add((Void) -> BuildingBlocks.eject());
			Initialization.switchSequence.add((Void) -> BuildingBlocks.moveUntilContact(-(84 - Initialization.robotDepth)/2, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
		}

		// Scale Sequence
		Initialization.scaleSequence.clear();

		if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition.equals("1")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(325.65 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(Initialization.autoScaleHeight));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(4, Initialization.autoMoveContactLow) & BuildingBlocks.eject());
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(-4, Initialization.autoMoveContactLow));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(12));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(0));

		} else if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition.equals("2")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(59 - Initialization.robotDepth, Initialization.autoMoveSpeed) & BuildingBlocks.autoElevatorHeight(62));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(173 + Initialization.robotWidth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(264.65, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(4, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(4, Initialization.autoMoveContactLow) & BuildingBlocks.eject());
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(-4, Initialization.autoMoveContactLow));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(12));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(0));

		} else if (Initialization.scalePosition == 'L' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(242.375 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(276 - Initialization.robotWidth, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(92.65, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(Initialization.autoScaleHeight));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(4, Initialization.autoMoveContactLow));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.eject());
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(1));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(-4, Initialization.autoMoveContactLow));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(12));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(0));

		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition.equals("1")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(242.375 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(264 - Initialization.robotWidth, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(83.65, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(Initialization.autoScaleHeight));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(4, Initialization.autoMoveContactLow));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.eject());
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(1));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(-4, Initialization.autoMoveContactLow));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(12));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(0));

		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition.equals("2")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(59 - Initialization.robotDepth, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(67 - Initialization.robotWidth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(4, Initialization.autoMoveContactLow) & BuildingBlocks.eject());
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(-4, Initialization.autoMoveContactLow));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(12));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(0));

		} else if (Initialization.scalePosition == 'R' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.switchSequence.add((Void) -> BuildingBlocks.delay(Initialization.finalDelay));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(323.65 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(Initialization.autoScaleHeight));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(4, Initialization.autoMoveContactLow) & BuildingBlocks.eject());
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(-4, Initialization.autoMoveContactLow));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(12));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.delay(0.5));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(0));
		}

		// Two Cube Sequence
		Initialization.twoCubeSequence.clear();

		if (Initialization.scalePosition == 'R' && Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition.equals("3")) {
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveAndBrake(325.65 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed) & BuildingBlocks.autoElevatorHeight(62));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(Initialization.autoScaleHeight));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(4, Initialization.autoMoveContactLow) & BuildingBlocks.eject());
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(-4, Initialization.autoMoveContactLow));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.scaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(0));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveAndBrake(84.915, Initialization.autoMoveSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveAndBrake(91.75 - (33.69 + Initialization.robotDepth / 2), Initialization.autoMoveSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveUntilContact(32.735, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.loadCube());
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.resetGrabber());
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.eject());
			
		} else if (Initialization.scalePosition == 'L' && Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition.equals("1")) {
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveAndBrake(325.65 - Initialization.robotDepth / 2, Initialization.autoMoveSpeed) & BuildingBlocks.autoElevatorHeight(62));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.delay(1));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.delay(1));// replace with end effector deposition code
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveAndBrake(84.915, Initialization.autoMoveSpeed) & BuildingBlocks.autoElevatorHeight(30));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotateAndBrake(-90, Initialization.autoRotateSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveAndBrake(91.75 - (33.69 + Initialization.robotDepth / 2), Initialization.autoMoveSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.rotateAndBrake(90, Initialization.autoRotateSpeed));
			Initialization.twoCubeSequence.add((Void) -> BuildingBlocks.moveUntilContact(32.735, Initialization.autoMoveContactHigh, Initialization.autoMoveContactLow));
		}
		
		// Side Scale Sequence
		Initialization.sideScaleSequence.clear();
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(196-Initialization.robotDepth, 0.7));
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.bankingRotate(Math.toDegrees(Math.atan((56.5-Initialization.robotWidth/2)/80)), 0.35));
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(Math.sqrt(Math.pow(56.5-Initialization.robotWidth/2, 2) + Math.pow(80, 2)), 0.45));
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.bankingRotate(-Math.toDegrees(Math.atan((56.5-Initialization.robotWidth/2)/80)), 0.35));
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(62));
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.delay(3));
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.autoElevatorHeight(0));
		Initialization.sideScaleSequence.add((Void) -> BuildingBlocks.moveAndBrake(-12, 0.45));
	}
}