package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class BuildingBlocks {
	
	public static boolean eject() {
		Initialization.endEffector.set(true);
		if(delay(1)) {
			Initialization.endEffector.set(false);
			return true;
		}
		return false;
	}
	
	public static boolean move(double dist, double speed) {
		if (dist > 18) {
			if (Math.abs(Initialization.leftDriveEncoder.getDistance()) < (Math.abs(dist) - 12)) {
				if (dist > 0) {
					Initialization.gearaffesDrive.arcadeDrive(speed, Initialization.gearaffesPID.get());																					
				} else {
					Initialization.gearaffesDrive.arcadeDrive(-speed, Initialization.gearaffesPID.get());
				}
				Robot.displaySensors();
				return false;
			} else {
				Initialization.gearaffesDrive.arcadeDrive(0, 0);
				return true;
			}
		} else {
			if (Math.abs(Initialization.leftDriveEncoder.getDistance()) < (Math.abs(dist))) {
				if (dist > 0) {
					Initialization.gearaffesDrive.arcadeDrive(Initialization.autoMoveContactLow,
							Initialization.gearaffesPID.get());
				} else {
					Initialization.gearaffesDrive.arcadeDrive(-Initialization.autoMoveContactLow,
							Initialization.gearaffesPID.get());
				}
				Robot.displaySensors();
				return false;
			} else {
				Initialization.gearaffesDrive.arcadeDrive(0, 0);
				
				return true;
			}
		}
	}

	public static int successesContact = 0;
	
	public static boolean moveUntilContact(double dist, double highSpeed, double lowSpeed) {
		Robot.displaySensors();
		if (successesContact < 5) {
			double speed = Math.abs(Initialization.leftDriveEncoder.getDistance()) < 0.8 * dist ? highSpeed : lowSpeed;
			Initialization.gearaffesDrive.arcadeDrive(speed, -Initialization.gyro.getAngle() * Initialization.move_KP);
			if (Math.abs(Initialization.leftDriveEncoder.getRate()) <= 0.5) {
				successesContact++;
			}
			return false;
		} else {
			Initialization.gearaffesDrive.arcadeDrive(0, 0);
			return true;
		}
	}

	public static boolean rotate(double angle, double power) {
		//if (Math.abs(Initialization.gyro.getAngle() * Initialization.MultiplierForGyro) < Math.abs(angle)) {
			Robot.displaySensors();
			if (angle > Initialization.gyro.getAngle() + 5) {
				Initialization.gearaffesDrive.arcadeDrive(0, power);
				return false;
			} else if(angle < Initialization.gyro.getAngle() - 5) {
				Initialization.gearaffesDrive.arcadeDrive(0, -power);
				return false;
			} else {
				Initialization.gearaffesDrive.arcadeDrive(0, 0);
				Initialization.gearaffesPID.gSource.setBaseAngle(angle);
				return true;
			}
		//} else {
			
		//}
	}
	
	public static boolean bankingRotate(double angle, double power) {
		if (Math.abs(Initialization.gyro.getAngle() * Initialization.MultiplierForGyro) < Math.abs(angle)) {

			if (angle > 0) {
				Initialization.FL.set(power);
				Initialization.BL.set(power);
			} else {
				Initialization.FR.set(-power);
				Initialization.BR.set(-power);
			}
			return false;
		} else {
			Initialization.FR.set(0);
			Initialization.BR.set(0);
			Initialization.FL.set(0);
			Initialization.BL.set(0);
			return true;
		}
	}
	
	public static void resetSomeSensors() {
		Initialization.leftDriveEncoder.reset();
		Initialization.rightDriveEncoder.reset();
		//Initialization.gyro.reset();
	}
	
	public static boolean autoElevatorHeight(double height) {
		boolean goSlowBottom = (Math.abs(Initialization.elevatorEncoder.getDistance()) < 12 && Math.signum(Initialization.elevator.get()) == -1);
		boolean goSlowTop = (Math.abs(Initialization.elevatorEncoder.getDistance()) > 72 && Math.signum(Initialization.elevator.get()) == 1);
		double speed = goSlowBottom ? Initialization.automationBottomSpeed : goSlowTop ? Initialization.automationTopSpeed : Initialization.automationHighSpeed;
		return setElevatorHeight(height, speed);
	}
	
	public static boolean setElevatorHeight(double height, double speed) {
		double dist = height - Math.abs(Initialization.elevatorEncoder.getDistance());
		if (height == 0) {
			return elevatorToBase(speed);
		} else if (dist > 0) {
			if (Math.abs(Initialization.elevatorEncoder.getDistance()) < height - 1) {
				Initialization.elevator.set(speed);
				return false;
			} else {
				Initialization.elevator.set(Initialization.automationHoldSpeed); // normally 0
				Teleop.automationInProgress = false;
				Teleop.startElevatorRumble(0.5, false, true);
				return true;
			}
		} else if (dist < 0) {
			if (Math.abs(Initialization.elevatorEncoder.getDistance()) > height + 1) {
				Initialization.elevator.set(-speed);
				return false;
			} else {
				Initialization.elevator.set(Initialization.automationHoldSpeed); //normally 0
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
	
	public static double endTime;
	public static boolean delayInProgress = false;
	
	public static boolean delay(double delay) {
		if (!delayInProgress) {
			endTime = Timer.getFPGATimestamp() + delay;
			delayInProgress = true;
			return false;
		} else {
			if(Timer.getFPGATimestamp() >= endTime) {
				delayInProgress = false;
				return true;
			} else {
				return false;
			}
		}
	}

	public static boolean elevatorToBase(double speed) {
		if (!Initialization.bottomLimitSwitch.get()) {
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

	public static void getMatchData() {
		try {
			Initialization.ourSwitchPosition = DriverStation.getInstance().getGameSpecificMessage().charAt(0);
			Initialization.scalePosition = DriverStation.getInstance().getGameSpecificMessage().charAt(1);
			Initialization.opposingSwitchPosition = DriverStation.getInstance().getGameSpecificMessage().charAt(2);
		} catch (NullPointerException e) {
			System.err.println("One or more of the field element positions could not be determined");
		}
	}
}
