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
				//Initialization.endEffector.set(false);
				scaleAutomationInProgress = false;
				switchAutomationInProgress = true;
			} else if (switchAutomationInProgress) {
				scaleAutomationInProgress = false;
				switchAutomationInProgress = false;
			}
		} else if (Initialization.driver.getRawButtonPressed(2)) {
			if (!scaleAutomationInProgress) {
				//Initialization.endEffector.set(false);
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
			//ejectCube();
			//intakeOutput();
			//grabber();
		}
	}

	public static boolean isDrivingOnGyro;
	
	public static void drive() {
		double finalMoveMultiplier = Initialization.driver.getRawButton(5) ? 1: Initialization.driver.getRawAxis(2) > 0.8 ? 0.5 : Initialization.moveMultiplier;
		double finalRotateMultiplier = Initialization.driver.getRawButton(6) ? 1: Initialization.driver.getRawAxis(3) > 0.8 ? 0.5 : Initialization.rotateMultiplier;
		/*if (Math.abs(Initialization.driver.getRawAxis(4)) < 0.05 && Math.abs(Initialization.driver.getRawAxis(1)) > 0.1 && !isDrivingOnGyro) {
			Initialization.gyro.reset();
			Initialization.gearaffesPID.reset();
			Initialization.gearaffesPID.enable();
			isDrivingOnGyro = true;
		} else if ((Math.abs(Initialization.driver.getRawAxis(4)) > 0.05 || Math.abs(Initialization.driver.getRawAxis(1)) < 0.1) && isDrivingOnGyro) {
			isDrivingOnGyro = false;
		}
		if (isDrivingOnGyro && Math.abs(Initialization.driver.getRawAxis(1)) > 0.05) {
			Initialization.gearaffesDrive.arcadeDrive(Math.signum(Initialization.driver.getRawAxis(1)) * -finalMoveMultiplier * Math.pow(Initialization.driver.getRawAxis(1), 2), Initialization.gearaffesPID.get());
		} else {*/
			Initialization.gearaffesDrive.arcadeDrive(Math.signum(Initialization.driver.getRawAxis(1)) * -finalMoveMultiplier * Math.pow(Initialization.driver.getRawAxis(1), 2), Math.signum(Initialization.driver.getRawAxis(4)) * finalRotateMultiplier* Math.pow(Initialization.driver.getRawAxis(4), 2), false);
		//}
		SmartDashboard.putNumber("Robot Speedometer", Robot.formatValue(Math.abs(Initialization.leftDriveEncoder.getRate()) / 12));
	}

	static boolean automationInProgress = false;
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
		if (automationInProgress) {
			double speed = goSlowBottom ? Initialization.automationBottomSpeed: goSlowTop ? Initialization.automationTopSpeed : Initialization.automationHighSpeed;
			BuildingBlocks.setElevatorHeight(elevatorTargetHeight, speed);

		} else if (Initialization.operator.getRawButtonPressed(1)) { // A
			automationInProgress = true;
			elevatorTargetHeight = 0; // 0 feet

		} else if (Initialization.operator.getRawButtonPressed(2)) { // B
			automationInProgress = true;
			elevatorTargetHeight = 24; // 2 feet

		} else if (Initialization.operator.getRawButtonPressed(4)) { // X
			automationInProgress = true;
			elevatorTargetHeight = 48; // 4 feet

		} else if (Initialization.operator.getRawButtonPressed(3)) { // Y
			automationInProgress = true;
			elevatorTargetHeight = 72; // 6 feet

		} else {
			double operatorOutput = -Math.signum(Initialization.operator.getRawAxis(1)) * Initialization.elevateMultiplier * Math.pow(Initialization.operator.getRawAxis(1), 2);
			if (goSlowBottom && Math.abs(operatorOutput) > Initialization.automationBottomSpeed) {
				Initialization.elevator.set(Math.signum(operatorOutput) * Initialization.automationBottomSpeed);
			} else if (goSlowTop && Math.abs(operatorOutput) > Initialization.automationTopSpeed) {
				Initialization.elevator.set(Math.signum(operatorOutput) * Initialization.automationTopSpeed);
			} else {
				Initialization.elevator.set(operatorOutput);
			}
		}

		SmartDashboard.putNumber("Elevator Height", Robot.formatValue(Math.abs(Initialization.elevatorEncoder.getDistance()) / 12));
	}

	public static void climb() {
		
	}

	/*public static void ejectCube() {
		Initialization.endEffector.set(Initialization.operator.getRawButton(5)); // Left Bumper
	}
	
	public static double grabberCount = 0;
	
	public static void grabber() {
		if(Initialization.operator.getRawButtonPressed(6)) { // right bumper
			grabberCount++;
		}
		if(grabberCount % 2 == 0) {
			Initialization.grabber.set(true);
		} else {
			Initialization.grabber.set(false);
		}
	}
	
	public static void intakeOutput() {
		if(Initialization.driver.getRawAxis(2) > 0.8){ // left axis underneath left bumper
			Initialization.intakePiston.set(true);
			Initialization.intakeMotor.set(-1); //
		} else if(Initialization.driver.getRawAxis(3) > 0.8) { // right axis underneath left bumper
			Initialization.intakePiston.set(true);
			Initialization.intakeMotor.set(1);
		} else {
			Initialization.intakePiston.set(false);
			Initialization.intakeMotor.set(0);
		}
	}*/

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