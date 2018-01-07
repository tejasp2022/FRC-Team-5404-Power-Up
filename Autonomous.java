/***************************************************************************************
*    Title: Gearaffes Robotics Team: FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous {
	
	
	public static void move(double dist, double speed) {
		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.gyro.reset();
		while (Math.abs(Initialization.rightDriveEncoder.getDistance()) < Math.abs(dist)) {
			//SmartDashboard.putNumber("GyroAngle", Initialization.gyro.getAngle());SmartDashboard.putNumber("RightEncoderDist", Initialization.rightDriveEncoder.getDistance());SmartDashboard.putNumber("LeftEncoderDist", Initialization.leftDriveEncoder.getDistance());
			if (dist > 0) {
				Initialization.gearaffesDrive.arcadeDrive(speed, Initialization.gyro.getAngle() * Initialization.move_KP);
			} else {
				Initialization.gearaffesDrive.arcadeDrive(-speed, Initialization.gyro.getAngle() * Initialization.move_KP);
			}
		}
		Initialization.gearaffesDrive.arcadeDrive(0, -Initialization.gyro.getAngle() * Initialization.move_KP);
		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.gyro.reset();
	}
	
	
	public static void rotate(double angle, double power) {
		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.gyro.reset();
		while (Math.abs(Initialization.gyro.getAngle() * Initialization.MultiplierForGyro) < Math.abs(angle)) {
			SmartDashboard.putNumber("GyroAngle", Initialization.gyro.getAngle());
			if (angle > 0) {
				Initialization.left.set(1 * power);
				Initialization.right.set(1 * power);
				
			} else {
				Initialization.left.set(-1 * power);
				Initialization.right.set(-1 * power);
			}
		}
		Initialization.left.set(0);
		Initialization.right.set(0);

		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.gyro.reset();
	}
	
	public static void crossBaseline() {
		// auto line is 10 feet from the alliance wall, so move forward 10.5 feet = 126 inches
		move(126, 0.7);
	}
	//center of switches are 14 ft from alliance wall, switch panels are 4 ft deep, so edge of switch should be 16 ft from wall
	//switch fence is 12 ft 9.5 in long
	public static void placeCubeOnSwitch() {
		if(Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition ==1) {
			
		} else if(Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition ==2) {
			
		} else if(Initialization.ourSwitchPosition == 'L' && Initialization.robotStartingPosition ==3) {
	
		} else if(Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition ==1) {
			
		} else if(Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition ==2) {
			
		} else if(Initialization.ourSwitchPosition == 'R' && Initialization.robotStartingPosition ==3) {
	
		}
	}
	
	public static void placeCubeOnScale() {
		if(Initialization.scalePosition == 'L' && Initialization.robotStartingPosition ==1) {
			
		} else if(Initialization.scalePosition == 'L' && Initialization.robotStartingPosition ==2) {
			
		} else if(Initialization.scalePosition == 'L' && Initialization.robotStartingPosition ==3) {
	
		} else if(Initialization.scalePosition == 'R' && Initialization.robotStartingPosition ==1) {
			
		} else if(Initialization.scalePosition == 'R' && Initialization.robotStartingPosition ==2) {
			
		} else if(Initialization.scalePosition == 'R' && Initialization.robotStartingPosition ==3) {
	
		}
	}
}
