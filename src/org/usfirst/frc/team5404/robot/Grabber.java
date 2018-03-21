package org.usfirst.frc.team5404.robot;

public class Grabber {
	
	public Grabber() {
		Initialization.cubeToEndEffectorSequence.add((Void) -> setGrabberState(true));
		Initialization.cubeToEndEffectorSequence.add((Void) -> DriveBase.moveAndBrake(5, -0.5));
		Initialization.cubeToEndEffectorSequence.add((Void) -> setGrabberPosition(Prefs.getDouble("Grabber Preset High", 150), grabberUpSpeed, grabberDownSpeed));
		Initialization.cubeToEndEffectorSequence.add((Void) -> setGrabberState(false));
		Initialization.cubeToEndEffectorSequence.add((Void) -> setGrabberPosition(0, grabberUpSpeed, grabberDownSpeed) &  DriveBase.moveAndBrake(5, 0.5) );
		
		Initialization.cubeToExtendSequence.add((Void) -> setGrabberState(true));
		Initialization.cubeToExtendSequence.add((Void) -> DriveBase.moveAndBrake(5, -0.5));
		Initialization.cubeToExtendSequence.add((Void) -> setGrabberPosition(Prefs.getDouble("Grabber Preset Medium", 90), grabberUpSpeed, grabberDownSpeed));
		Initialization.cubeToExtendSequence.add((Void) -> DriveBase.moveAndBrake(5, 0.5));
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
	
	public static int restorationCount = 0;

	public static boolean restoreGrabber() {
		if (Initialization.grabberPiston.get() && Initialization.grabberEncoder.getDistance() >= 80 && Math.abs(Initialization.rightDriveEncoder.getRate()) >= 5) {
			restorationCount++;
		}
		if (restorationCount >= 5) {
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
				restorationCount = 0;
				return true;
			}
		} else {
			Teleop.grabberAutomationInProgress = false;
			return true;
		}
	}
	
	public static int cubeToEndEffectorProcess = 0;

	public static boolean cubeToEndEffector() {
		if (cubeToEndEffectorProcess < Initialization.cubeToEndEffectorSequence.size()) {
			if (Initialization.cubeToEndEffectorSequence.get(cubeToEndEffectorProcess).apply(null)) {
				BuildingBlocks.resetSomeSensors();
				cubeToEndEffectorProcess++;
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

	public static int cubeToExtendProcess = 0;

	public static boolean cubeToExtend() {
		if (cubeToExtendProcess < Initialization.cubeToExtendSequence.size()) {
			if (Initialization.cubeToExtendSequence.get(cubeToExtendProcess).apply(null)) {
				BuildingBlocks.resetSomeSensors();
				cubeToExtendProcess++;
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
}
