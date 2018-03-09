package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class BuildingBlocks {
	
	public static void resetSomeSensors() {
		Initialization.leftDriveEncoder.reset();
		Initialization.rightDriveEncoder.reset();
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
