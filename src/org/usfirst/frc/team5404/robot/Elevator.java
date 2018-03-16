package org.usfirst.frc.team5404.robot;

public class Elevator {
	
	public static boolean eject() {
		Initialization.endEffectorPiston.set(true);
		if(BuildingBlocks.delay(1)) {
			Initialization.endEffectorPiston.set(false);
			return true;
		}
		return false;
	}
	public static boolean endeffOut() {
		Initialization.endEffectorPiston.set(true);
		return true;
	}
	public static boolean endeffIn() {
		Initialization.endEffectorPiston.set(false);
		return true;
	}
	
	public static boolean autoElevatorHeight(double height) {
		boolean goSlowBottom = (Math.abs(Initialization.elevatorEncoder.getDistance()) < 12 && Math.signum(Initialization.elevator.get()) == -1);
		boolean goSlowTop = (Math.abs(Initialization.elevatorEncoder.getDistance()) > 72 && Math.signum(Initialization.elevator.get()) == 1);
		double speed = goSlowBottom ? Initialization.automationBottomSpeed : goSlowTop ? Initialization.automationTopSpeed : Initialization.automationHighSpeed;
		return setElevatorHeight(height, speed);
	}
	
	public static boolean setElevatorHeight(double height, double speed) {
		double dist = height - Math.abs(Initialization.elevatorEncoder.getDistance());
		double effSpeed = Teleop.calculateElevatorOutput(speed);
		double nEffSpeed = Teleop.calculateElevatorOutput(-speed);
		if (height == 0) {
			return elevatorToBase(effSpeed);
		} else if (dist > 0) {
			if (Math.abs(Initialization.elevatorEncoder.getDistance()) < height - 1) {
				Initialization.elevator.set(effSpeed);
				return false;
			} else {
				Initialization.elevator.set(Initialization.automationElevatorHoldSpeed);
				Teleop.elevatorAutomationInProgress = false;
				Teleop.startElevatorRumble(0.5, false, true);
				return true;
			}
		} else if (dist < 0) {
			if (Math.abs(Initialization.elevatorEncoder.getDistance()) > height + 1) {
				Initialization.elevator.set(nEffSpeed);
				return false;
			} else {
				Initialization.elevator.set(Initialization.automationElevatorHoldSpeed);
				Teleop.elevatorAutomationInProgress = false;
				Teleop.startElevatorRumble(0.5, false, true);
				return true;
			}
		} else {
			Teleop.elevatorAutomationInProgress = false;
			Teleop.startElevatorRumble(0.5, false, true);
			return true;
		}
	}
	
	public static boolean elevatorToBase(double speed) {
		if (Math.abs(Initialization.elevatorEncoder.getDistance()) <= 0.5 || !Initialization.bottomLimitSwitch.get()) {
			Initialization.elevator.set(0);
			Teleop.elevatorAutomationInProgress = false;
			Teleop.startElevatorRumble(0.5, false, true);
			return true;
		} else {
			Initialization.elevator.set(-speed);
			Teleop.elevatorAutomationInProgress = true;
			return false;
		}
	}
	
	
}
