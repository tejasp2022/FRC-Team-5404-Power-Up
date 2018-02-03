/***************************************************************************************
*    Title: Gearaffes Robotics Team FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/                      
package org.usfirst.frc.team5404.robot;

import java.text.DecimalFormat;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	public static int autoProcess = 0;
	
	public void robotInit() {
		Initialization.autoModeTable = NetworkTableInstance.getDefault().getTable("Automode");
		Initialization.gearaffesDrive.setSafetyEnabled(false);
		resetSensors();
		calibrateSensors();
	}

	public static String lastTime = "0";

	public void robotPeriodic() {
		NetworkTableEntry sendEntry = Initialization.autoModeTable.getEntry("sendkey");
		NetworkTableValue val = sendEntry.getValue();
		if (val.getType() != NetworkTableType.kStringArray || val.getStringArray() == null
				|| val.getStringArray().length != 2) {
			sendEntry.setStringArray(new String[] { "0", "10000" });
		}
		String[] sArray = sendEntry.getValue().getStringArray();
		if (!sArray[0].equals(lastTime)) {
			String autoCode = sendEntry.getValue().getStringArray()[1];
			Initialization.robotStartingPosition = autoCode.substring(0, 1);
			Initialization.RLRStrat = autoCode.substring(1, 2);
			Initialization.LLLStrat = autoCode.substring(2, 3);
			Initialization.RRRStrat = autoCode.substring(3, 4);
			Initialization.LRLStrat = autoCode.substring(4, 5);
			Initialization.prefs.putString("Autonomous Code", autoCode);
			Initialization.autoModeTable.getEntry("receivekey").setStringArray(sArray);
		}
		try {
			lastTime = sArray[0];
		} catch (Exception e) {
		}
	}
	
	public void autonomousInit() {
		Initialization.gearaffesDrive.setSafetyEnabled(false);
		assignPreferenceVariables();
		Autonomous.getMatchData();
		Autonomous.determineAutonomousSequence();
		autoProcess = 0;
		resetSensors();
		calibrateSensors();
		Initialization.gearaffesPID = new GearaffesPID(Initialization.move_KP, Initialization.move_KI, Initialization.gyro, new GearaffesPID.GearaffesOutput());
		Initialization.gearaffesPID.enable();
		moveCalled = false;
	}
		boolean moveCalled = false;
	public void autonomousPeriodic() {
		/*if(!moveCalled) {
			if(Autonomous.move(Initialization.prefs.getDouble("Test Auto Distance", 10), Initialization.prefs.getDouble("Test Auto Input", 0.9))) {
			moveCalled = true; 
			}		
		}*/
		String union = Character.toString(Initialization.ourSwitchPosition) + Character.toString(Initialization.scalePosition);
		
		if(union.equalsIgnoreCase("RL")) {
			if (Initialization.RLRStrat.equals("1")) {
				Autonomous.crossAutoline();
			} else if (Initialization.RLRStrat.equals("2")) {
				Autonomous.placeCubeOnSwitch();
			} else if (Initialization.RLRStrat.equals("3")) {
				Autonomous.placeCubeOnScale();
			}		
		} else if(union.equalsIgnoreCase("LL")) {
			if (Initialization.LLLStrat.equals("1")) {
				Autonomous.crossAutoline();
			} else if (Initialization.LLLStrat.equals("2")) {
				Autonomous.placeCubeOnSwitch();
			} else if (Initialization.LLLStrat.equals("3")) {
				Autonomous.placeCubeOnScale();
			} else if (Initialization.LLLStrat.equals("4")) {
				Autonomous.twoCubeAuto();
			}
		} else if(union.equalsIgnoreCase("RR")) {
			if (Initialization.RRRStrat.equals("1")) {
				Autonomous.crossAutoline();
			} else if (Initialization.RRRStrat.equals("2")) {
				Autonomous.placeCubeOnSwitch();
			} else if (Initialization.RRRStrat.equals("3")) {
				Autonomous.placeCubeOnScale();
			} else if (Initialization.RRRStrat.equals("4")) {
				Autonomous.twoCubeAuto();
			}	
		} else if(union.equalsIgnoreCase("LR")) {
			if (Initialization.LRLStrat.equals("1")) {
				Autonomous.crossAutoline();
			} else if (Initialization.LRLStrat.equals("2")) {
				Autonomous.placeCubeOnSwitch();
			} else if (Initialization.LRLStrat.equals("3")) {
				Autonomous.placeCubeOnScale();
			}
		} else {
			SmartDashboard.putString("Autonomous Alert", "Valid Autonomous Strategy Not Found");
			System.err.println("Valid Autonomous Strategy Not Found");
		}
	}

	public void teleopInit() {
		Initialization.gearaffesDrive.setSafetyEnabled(false);
		assignPreferenceVariables();
		resetSensors();
		calibrateSensors();
	}
	
	public void teleopPeriodic() {
		Teleop.drive();
		Teleop.elevate();
		Teleop.ejectCube();
		Teleop.endGameRumble();
		Teleop.rangeDistance();
		Teleop.elevatorRumble();
		//Teleop.climb();
	}

	public void testInit() {
		Initialization.gearaffesDrive.setSafetyEnabled(false);
		assignPreferenceVariables();
		resetSensors();
		calibrateSensors();
		Test.testSequenceIndex = 0;
		Test.determineTestSequence();
	}
	
	public void testPeriodic() {
		if(SmartDashboard.getBoolean("Run Test Sequence", false)) { // Two-step verification so the robot doesn't randomly drive on the ground
			Test.runTestSequence();
		}
	}
	
	public void disabledInit() {
		Initialization.gearaffesPID.reset();
	}
	
	public void disabledPeriodic() {
		
	}
	
	public static void resetSensors() {
		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.elevatorEncoder.reset();
		Initialization.gyro.reset();
	}
	public void calibrateSensors() {
		Initialization.elevatorEncoder.setDistancePerPulse(Initialization.ELEVATOR_INCHES_PER_TICK);
		Initialization.rightDriveEncoder.setDistancePerPulse(Initialization.DRIVE_INCHES_PER_TICK);
		Initialization.leftDriveEncoder.setDistancePerPulse(Initialization.DRIVE_INCHES_PER_TICK);
		Initialization.elevatorEncoder.setSamplesToAverage(5);
		Initialization.rightDriveEncoder.setSamplesToAverage(1);
		Initialization.leftDriveEncoder.setSamplesToAverage(1);
	}
	
	public void setAutonFalse() {
		
	}
	
	public static double formatValue(double value) {
		return Double.valueOf(new DecimalFormat("###.00").format(value));
	}
	
	public static void displaySensors() {
		SmartDashboard.putNumber("Left Encoder Distance", formatValue(Math.abs(Initialization.leftDriveEncoder.getDistance())));
		SmartDashboard.putNumber("Right Encoder Distance", formatValue(Math.abs(Initialization.rightDriveEncoder.getDistance())));
		SmartDashboard.putNumber("Gyro Angle", formatValue(Initialization.gyro.getAngle()));
	}
	
	public static void assignPreferenceVariables() {
		Initialization.autoStrat = Initialization.prefs.getString("Autonomous Strategy", "Not Found");
		
		// Autonomous Moving Speeds
		Initialization.autoMoveSpeed = Initialization.prefs.getDouble("Auto Move Speed", 70)/100;
		Initialization.autoMoveContactHigh = Initialization.prefs.getDouble("Auto Move Contact High", 55)/100;	
		Initialization.autoMoveContactLow = Initialization.prefs.getDouble("Auto Move Contact Low", 30)/100;					
		Initialization.autoRotateSpeed = Initialization.prefs.getDouble("Auto Rotate Speed", 50)/100;					
		
		// Automation Moving Speeds
		Initialization.autoDelayTime = Initialization.prefs.getDouble("Autonomous Delay", 0);
		Initialization.automationHighSpeed = Initialization.prefs.getDouble("Automation High Speed", 70)/100;
		Initialization.automationTopSpeed = Initialization.prefs.getDouble("Automation Top Speed", 40)/100;
		Initialization.automationBottomSpeed = Initialization.prefs.getDouble("Automation Bottom Speed", 10)/100;
		
		// Robot Starting Position
		Initialization.robotStartingPosition = Initialization.prefs.getString("Autonomous Code", "-----").substring(0, 1);
		Initialization.RLRStrat = Initialization.prefs.getString("Autonomous Code", "-----").substring(1, 2);
		Initialization.LLLStrat = Initialization.prefs.getString("Autonomous Code", "-----").substring(2, 3);
		Initialization.RRRStrat = Initialization.prefs.getString("Autonomous Code", "-----").substring(3, 4);
		Initialization.LRLStrat = Initialization.prefs.getString("Autonomous Code", "-----").substring(4, 5);
		
		// Teleop Multipliers
		Initialization.moveMultiplier = Initialization.prefs.getDouble("Move Multiplier", 70)/100;
		Initialization.rotateMultiplier = Initialization.prefs.getDouble("Rotate Multiplier", 50)/100;
		Initialization.elevateMultiplier = Initialization.prefs.getDouble("Elevate Multiplier", 70)/100;
		
		// PID Multipliers
		Initialization.move_KP = Initialization.prefs.getDouble("Move KP", 0.06);
		Initialization.move_KI = Initialization.prefs.getDouble("Move KI", 0.03);
	}
}