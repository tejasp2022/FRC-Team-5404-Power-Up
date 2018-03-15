package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Grabber {
	
	public Grabber() {
		Initialization.grabberSequence.add((Void) -> setGrabberState(true));
		Initialization.grabberSequence.add((Void) -> DriveBase.moveAndBrake(5, -0.5));
		Initialization.grabberSequence.add((Void) -> setGrabberPosition(Prefs.getDouble("Grabber Preset High", 150), grabberUpSpeed, grabberDownSpeed));
		Initialization.grabberSequence.add((Void) -> setGrabberState(false));
		Initialization.grabberSequence.add((Void) -> setGrabberPosition(0, grabberUpSpeed, grabberDownSpeed) &  DriveBase.moveAndBrake(5, 0.5) );
	}
	
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
	
	
	public static int grabberProcess = 0;

	public static boolean cubeToEndEffectorGood() {
		if (grabberProcess < Initialization.grabberSequence.size()) {
			if (Initialization.grabberSequence.get(grabberProcess).apply(null)) {
				BuildingBlocks.resetSomeSensors();
				grabberProcess++;
				DriveBase.successesContact = 0;
				Initialization.gearaffesPID.reset();
				Initialization.gearaffesPID.enable();
				DriveBase.setBraking(false);
			}
			return false;
		} else {
			return true;
		}
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
