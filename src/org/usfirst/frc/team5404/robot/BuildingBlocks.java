package org.usfirst.frc.team5404.robot;

import java.util.function.Function;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
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
		//double brakeKP = Initialization.brakeKP;
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
			if (Math.abs(Initialization.leftDriveEncoder.getDistance()) +
					(Initialization.brakeB0 +
							Math.abs(Initialization.leftDriveEncoder.getRate()) * Initialization.brakeB1)
						< Math.abs(dist)) {
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

	/**
	 * Braking power for rotation follows a PARABOLIC model: Predicted Overshoot = B0 + B1(Instantaneous Rate)^2
	 * @param angle
	 * @param power
	 * @return
	 */
	public static boolean rotateAndBrake(double angle, double power) {
		if(isBraking) {
			//double leftRate = Initialization.leftDriveEncoder.getRate();
			//double rightRate = Initialization.rightDriveEncoder.getRate();
			double gRate = Math.abs(Initialization.gyro.getRate());
			Initialization.gearaffesDrive.arcadeDrive(0, 0);
			/*Initialization.FL.set(-brakeKP*leftRate);
			Initialization.BL.set(-brakeKP*leftRate);
			Initialization.FR.set(-brakeKP*rightRate);
			Initialization.BR.set(-brakeKP*rightRate);*/
			setBraking(true);
			if(Math.abs(Initialization.gyro.getAngle()) >= Math.abs(angle) || gRate < 0.3) {
				isBraking = false;
				Initialization.gearaffesDrive.arcadeDrive(0, 0);
				setBraking(false);
				postRotate();
				return true;
			} else {
				return false;
			}
		} else {
			double B0 = Prefs.getDouble("Brake Rotation B0", 6.12944);
			double B1 = Prefs.getDouble("Brake Rotation B1", 0.000534458);
			double gRate = Math.abs(Initialization.gyro.getRate());
			double predictedError = B0 + B1 * gRate * gRate;
			if (Math.abs(Initialization.gyro.getAngle() * Initialization.MultiplierForGyro) + predictedError < Math.abs(angle)) {
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
	}
	
	public static boolean rotate(double angle, double power) {
		/*if(isBraking) {
			double leftRate = Initialization.leftDriveEncoder.getRate();
			double rightRate = Initialization.rightDriveEncoder.getRate();
			
			Initialization.gearaffesDrive.arcadeDrive(0, 0);
			/*Initialization.FL.set(-brakeKP*leftRate);
			Initialization.BL.set(-brakeKP*leftRate);
			Initialization.FR.set(-brakeKP*rightRate);
			Initialization.BR.set(-brakeKP*rightRate);*//*
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
		} else */if (Math.abs(Initialization.gyro.getAngle() * Initialization.MultiplierForGyro) < Math.abs(angle) - 8) {
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
	
	public static boolean doGrabberRelease = false;
	
	public static boolean setGrabberPosition(double position, double upSpeed, double downSpeed) {
		double dist = position - Math.abs(Initialization.grabberEncoder.getDistance());
		if (dist > 0) {
			if (Math.abs(Initialization.grabberEncoder.getDistance()) < position - 5) {
				Initialization.grabberMotorController.set(Teleop.calculateGrabberOutput(upSpeed));
				return false;
			} else {
				Initialization.grabberMotorController.set(calculateGrabberHoldSpeed());
				Teleop.grabberAutomationInProgress = false;
				if(doGrabberRelease) {
					if(Initialization.grabberPiston.get()) {
						Initialization.grabberPiston.set(false);
						Teleop.grabberCount++;
						Teleop.grabberAutomationInProgress = true;
						Teleop.grabberTargetAngle = 0;
					}
				}
				doGrabberRelease = false;
				return true;
			}
		} else if (dist < 0) {
			if (Math.abs(Initialization.grabberEncoder.getDistance()) > position + 5) {
				Initialization.grabberMotorController.set(Teleop.calculateGrabberOutput(-downSpeed));
				return false;
			} else {
				Initialization.grabberMotorController.set(calculateGrabberHoldSpeed());
				Teleop.grabberAutomationInProgress = false;
				if(doGrabberRelease) {
					if(Initialization.grabberPiston.get()) {
						Initialization.grabberPiston.set(false);
						Teleop.grabberCount++;
						Teleop.grabberAutomationInProgress = true;
						Teleop.grabberTargetAngle = 0;
					}
				}
				doGrabberRelease = false;
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
	
	public static boolean loadCube() {//gets cube from ground onto end effector
		Initialization.grabberPiston.set(true);
		setGrabberPosition(150,0.8,0.6);
		if(Initialization.grabberPiston.get() && Initialization.grabberEncoder.getDistance() == 150) {
			Initialization.grabberPiston.set(false);
			return true;
		}
		return false;
	}
	
	public static boolean resetGrabber() {//brings grabber back down
		setGrabberPosition(0,0.8,0.6);
		if(Initialization.grabberEncoder.getDistance() == 0) {
			return true;
		}
		return false;
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
	public static boolean doFuncA = true, doFuncB = true;
	public static boolean concurrent(Function<Void, Boolean> funcA, Function<Void, Boolean> funcB) {
		if(doFuncA) {
			if(funcA.apply(null)) {
				doFuncA = false;
			}
		}
		if(doFuncB) {
			if(funcB.apply(null)) {
				doFuncB = false;
			}
		}
		if(!doFuncA && !doFuncB) {
			doFuncA = true;
			doFuncB = true;
			return true;
		}
		return false;
	}

	public static int tries = 0;
	public static void getMatchData() {
		while(DriverStation.getInstance().getGameSpecificMessage().length()<3){
			if(tries>=700) {
				break;
			}
			try {
				Initialization.ourSwitchPosition = DriverStation.getInstance().getGameSpecificMessage().charAt(0);
				Initialization.scalePosition = DriverStation.getInstance().getGameSpecificMessage().charAt(1);
				Initialization.opposingSwitchPosition = DriverStation.getInstance().getGameSpecificMessage().charAt(2);
			} catch (NullPointerException e) {
				System.err.println("One or more of the field element positions could not be determined");
			}
			tries++;
		}
	}
}
