package org.usfirst.frc.team5404.robot;

public class Grabber {
	
	public static double grabberUpSpeed = 0.8;
	public static double grabberDownSpeed = 0.6;
	
	public static boolean setGrabberState(boolean state) {
		Initialization.grabberPiston.set(state);
		if (BuildingBlocks.delay(0.5)) {
			return true;
		}
		return false;
	}
	
	public static double calculateGrabberHoldSpeed() {
		double hold = Initialization.grabberPiston.get() ? Initialization.automationGrabberCubeHoldSpeed : Initialization.automationGrabberHoldSpeed;
		return hold * Math.sin(Math.toRadians(Initialization.grabberEncoder.getDistance()));
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
	
	public static boolean cubeToEndEffector() {
        	if (setGrabberState(true)) {
            		if (DriveBase.moveAndBrake(5, -0.5)) {
                		if (setGrabberPosition(Prefs.getDouble("Grabber Preset High", 150), grabberUpSpeed, grabberDownSpeed)) {
                    			if (setGrabberState(false)) {
                       		 		if (setGrabberPosition(0, grabberUpSpeed, grabberDownSpeed)) {
                            				if (DriveBase.moveAndBrake(5, 0.5)) {
                                				return true;
                            				}
                        			}
                    			}
                		}
            		}
        	}
        return false;
    }
	
    	public static boolean cubeToExtend() {
        	if (setGrabberState(true)) {
            		if (DriveBase.moveAndBrake(5, -0.5)) {
                		if (setGrabberPosition(Prefs.getDouble("Grabber Preset Medium", 90), grabberUpSpeed, grabberDownSpeed)) {
                    			if (DriveBase.moveAndBrake(5, 0.5)) {
                        			return true;
                    			}
                		}
           		 }
        	}
        return false;
    }

}
