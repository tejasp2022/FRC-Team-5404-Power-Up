/***************************************************************************************
*    Title: Gearaffes Robotics Team FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Teleop {
	public static boolean switchAutomationInProgress = false;
	public static boolean scaleAutomationInProgress = false;

	public static void cubeManipulation() {
		if (Initialization.driver.getRawButtonPressed(1)) {
			if (!switchAutomationInProgress) {
				Initialization.endEffectorPiston.set(false);
				scaleAutomationInProgress = false;
				switchAutomationInProgress = true;
			} else if (switchAutomationInProgress) {
				scaleAutomationInProgress = false;
				switchAutomationInProgress = false;
			}
		} else if (Initialization.driver.getRawButtonPressed(2)) {
			if (!scaleAutomationInProgress) {
				Initialization.endEffectorPiston.set(false);
				switchAutomationInProgress = false;
				scaleAutomationInProgress = true;
			} else if (scaleAutomationInProgress) {
				scaleAutomationInProgress = false;
				switchAutomationInProgress = false;
			}
		}

		if (switchAutomationInProgress) {
			Autonomous.placeCubeOnSwitch();
		} else if (scaleAutomationInProgress) {
			Autonomous.placeCubeOnScale();
		} else {
			drive();
			elevate();
			ejectCube();
			grabber();
			//toeSucker();
		}
	}

	public static boolean isDrivingOnGyro;
	
	public static void drive() {
		if(Initialization.driver.getRawButton(3)) {
			BuildingBlocks.setBraking(true);
		} else {
			BuildingBlocks.setBraking(false);
		}
		double finalMoveMultiplier = Initialization.driver.getRawButton(5) ? 1: Initialization.driver.getRawAxis(2) > 0.8 ? 0.5 : Initialization.moveMultiplier;
		double finalRotateMultiplier = Initialization.driver.getRawButton(6) ? 1: Initialization.driver.getRawAxis(3) > 0.8 ? 0.5 : Initialization.rotateMultiplier;
		Initialization.gearaffesDrive.arcadeDrive(Math.signum(Initialization.driver.getRawAxis(1)) * -finalMoveMultiplier * Math.pow(Initialization.driver.getRawAxis(1), 2), Math.signum(Initialization.driver.getRawAxis(4)) * finalRotateMultiplier* Math.pow(Initialization.driver.getRawAxis(4), 2), false);
		SmartDashboard.putNumber("Robot Speedometer", Robot.formatValue(Math.abs(Initialization.leftDriveEncoder.getRate()) / 12));
	}

	static boolean elevatorAutomationInProgress = false;
	static double elevatorTargetHeight;
	
	public static void elevate() {
		if (!Initialization.topLimitSwitch.get() || !Initialization.bottomLimitSwitch.get()) {
			Initialization.elevator.set(0);
			if (!Initialization.bottomLimitSwitch.get()) {
				Initialization.elevatorEncoder.reset();
			}
		}
		boolean goSlowBottom = (Math.abs(Initialization.elevatorEncoder.getDistance()) < 12 && Math.signum(Initialization.elevator.get()) == -1);
		boolean goSlowTop = (Math.abs(Initialization.elevatorEncoder.getDistance()) > 72 && Math.signum(Initialization.elevator.get()) == 1);
		if (elevatorAutomationInProgress) {
			double speed = goSlowBottom ? Initialization.automationBottomSpeed: goSlowTop ? Initialization.automationTopSpeed : Initialization.automationHighSpeed;
			BuildingBlocks.setElevatorHeight(elevatorTargetHeight, speed);

		} else if (Initialization.operator.getRawButtonPressed(1)) { // A
			elevatorAutomationInProgress = true;
			elevatorTargetHeight = 0; // 0 feet

		} else if (Initialization.operator.getRawButtonPressed(2)) { // B
			elevatorAutomationInProgress = true;
			elevatorTargetHeight = 32; // 4 feet

		} else if (Initialization.operator.getRawButtonPressed(4)) { // y
			elevatorAutomationInProgress = true;
			elevatorTargetHeight = 47; // 5 feet

		} else if (Initialization.operator.getRawButtonPressed(3)) { // x
			elevatorAutomationInProgress = true;
			elevatorTargetHeight = 60; // 6 feet

		} else {
			double operatorOutput = -Initialization.elevateMultiplier * Initialization.operator.getRawAxis(5);
			if (goSlowBottom && Math.abs(operatorOutput) > Initialization.automationBottomSpeed) {
				Initialization.elevator.set(calculateElevatorOutput(Math.signum(operatorOutput) * Initialization.automationBottomSpeed));
			} else if (goSlowTop && Math.abs(operatorOutput) > Initialization.automationTopSpeed) {
				Initialization.elevator.set(calculateElevatorOutput(Math.signum(operatorOutput) * Initialization.automationTopSpeed));
			} else {
				Initialization.elevator.set(calculateElevatorOutput(operatorOutput));
			}
		}

		SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
	}
	public static double calculateElevatorOutput(double operatorOutput) {
		double effectiveHoldSpeed = Initialization.elevatorEncoder.getDistance() > 6 ? Initialization.automationElevatorHoldSpeed : 0;
		if(operatorOutput >= 0) {
			return effectiveHoldSpeed + (1 - effectiveHoldSpeed) * operatorOutput;
		} else {
			return effectiveHoldSpeed + (1 + effectiveHoldSpeed) * operatorOutput;
		}
	}


	/*public static void toeSucker() {
		Initialization.intakePiston1.set(false);
		Initialization.intakePiston2.set(true);
		if(Initialization.operator.getRawAxis(2) >= 0.8) {
			//Initialization.intakePiston1.set(false);
			//Initialization.intakePiston2.set(true);
			if(Initialization.operator.getRawButton(8)) {
				Initialization.intakeMotorControllerRight.set(-1);
				Initialization.intakeMotorControllerLeft.set(1);
			} else {
				Initialization.intakeMotorControllerRight.set(-1);
				Initialization.intakeMotorControllerLeft.set(-1);
			}
		} else {
			//Initialization.intakePiston1.set(true);
			//Initialization.intakePiston2.set(false);
			Initialization.intakeMotorControllerRight.set(0);
			Initialization.intakeMotorControllerLeft.set(0);
		}
	}*/
	// right on 6 left on 7
	public static void climb() {
		
	}

	public static void ejectCube() {
		Initialization.endEffectorPiston.set(Initialization.operator.getRawButton(6)); // right Bumper
	}
	
	public static double grabberCount = 0;
	public static boolean grabberAutomationInProgress = false;
	public static double grabberTargetAngle;
	
	public static void grabber() {
		if (grabberAutomationInProgress) {
			double upSpeed = 0.8;
			double downSpeed = 0.6;
			if(Initialization.operator.getPOV() == 270) {
				grabberAutomationInProgress = false;
			} else {
				BuildingBlocks.setGrabberPosition(grabberTargetAngle, upSpeed, downSpeed);
			}

		} else if( Initialization.operator.getPOV()!= -1) {
			if(Initialization.operator.getPOV() == 0) {
				grabberAutomationInProgress = true;
				grabberTargetAngle = Prefs.getDouble("Grabber Preset High", 150); 
				BuildingBlocks.doGrabberRelease = true;
				
			} else if (Initialization.operator.getPOV() == 90) {
				grabberAutomationInProgress = true;
				grabberTargetAngle = Prefs.getDouble("Grabber Preset Medium", 90); 
				BuildingBlocks.doGrabberRelease = false;
	
			} else if (Initialization.operator.getPOV() == 180) {
				grabberAutomationInProgress = true;
				grabberTargetAngle = Prefs.getDouble("Grabber Preset Low", 0);
				BuildingBlocks.doGrabberRelease = false;
				
			} else if (Initialization.operator.getPOV() == 270) {
				grabberAutomationInProgress = false;
			}
		} else {
			if(Initialization.operator.getRawButtonPressed(5)) { // left bumper
				grabberCount++;
				System.out.println(grabberCount);
			}
			if(grabberCount % 2 == 0) {
				Initialization.grabberPiston.set(false);
			} else {
				Initialization.grabberPiston.set(true);
			}
			double operatorOutput = -Math.signum(Initialization.operator.getRawAxis(1)) * Math.pow(Initialization.operator.getRawAxis(1), 2);
			Initialization.grabberMotorController.set(calculateGrabberOutput(operatorOutput));
		}
		
		

	}
	public static double calculateGrabberOutput(double operatorOutput) {
		double holdSpeed = BuildingBlocks.calculateGrabberHoldSpeed();
		if(operatorOutput >= 0) {
			return holdSpeed + (1 - holdSpeed) * operatorOutput;
		} else {
			return holdSpeed + (1 + holdSpeed) * operatorOutput;
		}
	}

	public static void rangeDistance() {
		SmartDashboard.putNumber("Range Finder Value", Initialization.rangeFinder.getVoltage() * 1000 / 25.4);
	}

	static boolean rumbleInProgress = false;
	public static double setTime;
	public static double endTime;
	public static boolean driver, operator;

	public static void startElevatorRumble(double duration, boolean driver, boolean operator) {
		setTime = Timer.getFPGATimestamp();
		endTime = setTime + duration;
		Teleop.driver = driver;
		Teleop.operator = operator;
		rumbleInProgress = true;
	}

	public static void elevatorRumble() {
		if (rumbleInProgress) {
			if (Timer.getFPGATimestamp() < endTime) {
				if (driver) {
					Initialization.driver.setRumble(RumbleType.kLeftRumble, 1);
					Initialization.driver.setRumble(RumbleType.kRightRumble, 1);
				}
				if (operator) {
					Initialization.operator.setRumble(RumbleType.kLeftRumble, 1);
					Initialization.operator.setRumble(RumbleType.kRightRumble, 1);
				}
			} else {
				Initialization.driver.setRumble(RumbleType.kLeftRumble, 0);
				Initialization.driver.setRumble(RumbleType.kRightRumble, 0);
				Initialization.operator.setRumble(RumbleType.kLeftRumble, 0);
				Initialization.operator.setRumble(RumbleType.kRightRumble, 0);
				rumbleInProgress = false;
			}
		}
	}

	public static void endGameRumble() {
		if ((Timer.getMatchTime() < 30 && Timer.getMatchTime() > 29.5) || (Timer.getMatchTime() < 29 && Timer.getMatchTime() > 28.5) || (Timer.getMatchTime() < 28 && Timer.getMatchTime() > 27.5)) {
			Initialization.driver.setRumble(RumbleType.kLeftRumble, 1);
			Initialization.driver.setRumble(RumbleType.kRightRumble, 1);
			Initialization.operator.setRumble(RumbleType.kLeftRumble, 1);
			Initialization.operator.setRumble(RumbleType.kRightRumble, 1);
		} else {
			Initialization.driver.setRumble(RumbleType.kLeftRumble, 0);
			Initialization.driver.setRumble(RumbleType.kRightRumble, 0);
			Initialization.operator.setRumble(RumbleType.kLeftRumble, 0);
			Initialization.operator.setRumble(RumbleType.kRightRumble, 0);
		}
	}
}