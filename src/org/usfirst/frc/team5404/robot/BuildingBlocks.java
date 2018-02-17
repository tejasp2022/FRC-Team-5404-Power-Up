package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class BuildingBlocks {
	public static boolean eject() {
		Initialization.endEffectorPiston.set(true);
		if(delay(1)) {
			Initialization.endEffectorPiston.set(false);
			return true;
		}
		return false;
	}
	public static void setBraking(boolean braking) {
		Initialization.brakePiston1.set(!braking);
		Initialization.brakePiston2.set(braking);
	}
	static boolean isBraking = false;
	public static boolean moveAndBrake(double dist, double speed) {
		double brakeKP = Initialization.brakeKP;
		//double brakeDist = Initialization.prefs.getDouble("Brake Distance", 22);
		if(isBraking) {
			double leftRate = Initialization.leftDriveEncoder.getRate();
			double rightRate = Initialization.rightDriveEncoder.getRate();
			
			Initialization.gearaffesDrive.arcadeDrive(0, 0);
			/*Initialization.FL.set(-brakeKP*leftRate);
			Initialization.BL.set(-brakeKP*leftRate);
			Initialization.FR.set(-brakeKP*rightRate);
			Initialization.BR.set(-brakeKP*rightRate);*/
			setBraking(true);
			if(Math.abs(leftRate) < 15 || Math.abs(rightRate) < 15) {
				isBraking = false;
				Initialization.gearaffesDrive.arcadeDrive(0, 0);
				setBraking(false);
				return true;
			} else {
				return false;
			}
		} else { // arbitrary but we can test later
			if (Initialization.leftDriveEncoder.getDistance() +
					(Initialization.brakeB0 +
							Initialization.leftDriveEncoder.getRate() * Initialization.brakeB1)
						< dist) {
				if (dist > 0) {
					Initialization.gearaffesDrive.arcadeDrive(speed, Initialization.gearaffesPID.get(), false);																					
				} else {
					Initialization.gearaffesDrive.arcadeDrive(-speed, Initialization.gearaffesPID.get(), false);
				}
				Robot.displaySensors();
				return false;
			} else {
				Initialization.gearaffesDrive.arcadeDrive(0, 0);
				postRotate();
				isBraking = true;
				return false;
			}
		} /*else {
			if (Math.abs(Initialization.leftDriveEncoder.getDistance()) < (Math.abs(dist))) {
				if (dist > 0) {
					Initialization.gearaffesDrive.arcadeDrive(Initialization.autoMoveContactLow,
							Initialization.gearaffesPID.get(), false);
				} else {
					Initialization.gearaffesDrive.arcadeDrive(-Initialization.autoMoveContactLow,
							Initialization.gearaffesPID.get(), false);
				}
				Robot.displaySensors();
				return false;
			} else {
				Initialization.gearaffesDrive.arcadeDrive(0, 0);
				return true;
			}
		}*/
	}
	public static boolean move(double dist, double speed) {
		if (dist > 18) {
			if (Math.abs(Initialization.leftDriveEncoder.getDistance()) < (Math.abs(dist) - Initialization.prefs.getDouble("Brake Distance", 12))) {
				if (dist > 0) {
					Initialization.gearaffesDrive.arcadeDrive(speed, Initialization.gearaffesPID.get(), false);																					
				} else {
					Initialization.gearaffesDrive.arcadeDrive(-speed, Initialization.gearaffesPID.get(), false);
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
							Initialization.gearaffesPID.get(), false);
				} else {
					Initialization.gearaffesDrive.arcadeDrive(-Initialization.autoMoveContactLow,
							Initialization.gearaffesPID.get(), false);
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
	public static double contactBegin = -1;
	
	public static boolean moveUntilContact(double dist, double highSpeed, double lowSpeed) {
		Robot.displaySensors();
		if(contactBegin == -1) {
			contactBegin = Timer.getFPGATimestamp();
		}
		if (successesContact < 5) {
			double speed = Math.abs(Initialization.leftDriveEncoder.getDistance()) < 0.8 * dist ? highSpeed : lowSpeed;
			Initialization.gearaffesDrive.arcadeDrive(speed, -Initialization.gyro.getAngle() * Initialization.move_KP, false);
			if (Math.abs(Initialization.leftDriveEncoder.getRate()) <= 5 && Timer.getFPGATimestamp() > contactBegin + 0.4) {
				successesContact++;
			}
			return false;
		} else {
			Initialization.gearaffesDrive.arcadeDrive(0, 0);
			contactBegin = -1;
			return true;
		}
	}

	public static boolean rotate(double angle, double power) {
		if(isBraking) {
			double leftRate = Initialization.leftDriveEncoder.getRate();
			double rightRate = Initialization.rightDriveEncoder.getRate();
			
			Initialization.gearaffesDrive.arcadeDrive(0, 0);
			/*Initialization.FL.set(-brakeKP*leftRate);
			Initialization.BL.set(-brakeKP*leftRate);
			Initialization.FR.set(-brakeKP*rightRate);
			Initialization.BR.set(-brakeKP*rightRate);*/
			setBraking(true);
			if(Math.abs(Initialization.gyro.getAngle()) >= angle) {
				isBraking = false;
				Initialization.gearaffesDrive.arcadeDrive(0, 0);
				setBraking(false);
				postRotate();
				return true;
			} else {
				return false;
			}
		} else if (Math.abs(Initialization.gyro.getAngle() * Initialization.MultiplierForGyro) < Math.abs(angle) - 8) {
			Robot.displaySensors();
			if (angle > 0) {
				Initialization.gearaffesDrive.arcadeDrive(0, power, false);
			} else {
				Initialization.gearaffesDrive.arcadeDrive(0, -power, false);
			}
			return false;
		} else {
			Initialization.gearaffesDrive.arcadeDrive(0, 0);
			//postRotate();
			setBraking(true);
			isBraking = true;
			return false;
		}
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
			postRotate();
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
	public static void postRotate() {
		Initialization.gyro.reset();
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
				Initialization.elevator.set(Initialization.automationElevatorHoldSpeed);
				Teleop.elevatorAutomationInProgress = false;
				Teleop.startElevatorRumble(0.5, false, true);
				return true;
			}
		} else if (dist < 0) {
			if (Math.abs(Initialization.elevatorEncoder.getDistance()) > height + 1) {
				Initialization.elevator.set(-speed);
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
	

	
	public static boolean setGrabberPosition(double position, double speed) {
		double dist = position - Math.abs(Initialization.grabberEncoder.getDistance());
		if (dist > 0) {
			if (Math.abs(Initialization.grabberEncoder.getDistance()) < position - 5) {
				Initialization.grabberMotorController.set(Teleop.calculateGrabberOutput(speed));
				return false;
			} else {
				Initialization.grabberMotorController.set(calculateGrabberHoldSpeed());
				Teleop.grabberAutomationInProgress = false;
				if(Initialization.grabberPiston.get()) {
					Initialization.grabberPiston.set(false);
					Teleop.grabberCount++;
					Teleop.grabberAutomationInProgress = true;
					Teleop.grabberTargetHeight = 0;
				}
				return true;
			}
		} else if (dist < 0) {
			if (Math.abs(Initialization.grabberEncoder.getDistance()) > position + 5) {
				Initialization.grabberMotorController.set(Teleop.calculateGrabberOutput(-speed));
				return false;
			} else {
				Initialization.grabberMotorController.set(calculateGrabberHoldSpeed());
				Teleop.grabberAutomationInProgress = false;
				return true;
			}
		} else {
			Teleop.grabberAutomationInProgress = false;
			return true;
		}
	}
	public static double calculateGrabberHoldSpeed() {
		double hold = Initialization.grabberPiston.get() ? Initialization.automationGrabberCubeHoldSpeed : Initialization.automationGrabberHoldSpeed;
		return hold * Math.sin(Math.toRadians(Initialization.grabberEncoder.getDistance()));
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
			Teleop.elevatorAutomationInProgress = false;
			Teleop.startElevatorRumble(0.5, false, true);
			return true;
		} else {
			Initialization.elevator.set(-speed);
			Teleop.elevatorAutomationInProgress = true;
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
