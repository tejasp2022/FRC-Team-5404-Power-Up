package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.Timer;

public class DriveBase {
	static boolean isBraking = false;
	public static boolean moveAndBrake(double dist, double speed) {
		if(isBraking) {
			double leftRate = Initialization.leftDriveEncoder.getRate();
			double rightRate = Initialization.rightDriveEncoder.getRate();
			Initialization.gearaffesDrive.arcadeDrive(0, 0);
			setBraking(true);
			if(Math.abs(leftRate) < 15 || Math.abs(rightRate) < 15) {
				isBraking = false;
				Initialization.gearaffesDrive.arcadeDrive(0, 0);
				setBraking(false);
				return true;
			} else {
				return false;
			}
		} else { 
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
		} 
	}
	
	
	public static void setBraking(boolean braking) {
		Initialization.brakePiston1.set(braking);
		//Initialization.brakePiston2.set(braking);
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
			double gRate = Math.abs(Initialization.gyro.getRate());
			Initialization.gearaffesDrive.arcadeDrive(0, 0);
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
		if (Math.abs(Initialization.gyro.getAngle() * Initialization.MultiplierForGyro) < Math.abs(angle) - 8) {
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
	
	public static void postRotate() {
		Initialization.gyro.reset();
	}
	
}
