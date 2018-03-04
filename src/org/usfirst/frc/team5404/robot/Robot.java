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
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;

public class Robot extends TimedRobot {

	public static int autoProcess = 0;

	public void robotInit() {
		Initialization.cam.startAutomaticCapture(0);
		Initialization.cam.startAutomaticCapture(1);
		Initialization.autoModeTable = NetworkTableInstance.getDefault().getTable("Automode");
		Initialization.gearaffesDrive.setSafetyEnabled(false);
		resetSensors();
		calibrateSensors();
		SmartDashboard.putBoolean("Record Robot", false);
		SmartDashboard.putBoolean("Playback Robot", false);
		SmartDashboard.putBoolean("Reverse Playback Robot", false);
	}

	public static String lastTime = "0";

	public void robotPeriodic() {
		Robot.displaySensors();
		
		NetworkTableEntry sendEntry = Initialization.autoModeTable.getEntry("sendkey");
		NetworkTableValue val = sendEntry.getValue();
		if (val.getType() != NetworkTableType.kStringArray || val.getStringArray() == null || val.getStringArray().length != 2) {
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

	public static int robotValuesI = 0;
	public static boolean needToPopulateTwoCube = true;
	public static boolean needToPopulateThreeCube = true;
	public void autonomousInit() {
		Initialization.gearaffesDrive.setSafetyEnabled(false);
		assignPreferenceVariables();
		BuildingBlocks.getMatchData();
		Autonomous.determineAutonomousSequence();
		autoProcess = 0;
		resetSensors();
		calibrateSensors();
		Initialization.gearaffesPID = new GearaffesPID(Initialization.move_KP, Initialization.move_KI, Initialization.gyro, new GearaffesPID.GearaffesOutput());
		Initialization.gearaffesPID.enable();
		rotateCalled = false;
		BuildingBlocks.successesContact = 0;
		BuildingBlocks.setBraking(false);
		//Autonomous.sendActionSummary();
		playbackI = 0;
		playbackReverseI = Autonomous.timeList.size()-1;
		needToPopulateTwoCube = true;
		needToPopulateThreeCube = true;
		robotValuesI = 0;
		MatchData.beginLogging(Mode.AUTO);
	}

	boolean rotateCalled = false;

	int playbackI = 0;
	int playbackReverseI = Autonomous.timeList.size()-1;
	public void autonomousPeriodic() {
		/*if(!rotateCalled) {
			if(BuildingBlocks.moveAndBrake(120, 0.9)) {
				rotateCalled = true;
			}
		}/**/
		
		if (SmartDashboard.getBoolean("Playback Robot", false)) {
			Autonomous.playback(playbackI++);
		
		} else if (SmartDashboard.getBoolean("Reverse Playback Robot", false)) {
			Autonomous.playbackReverse(playbackReverseI--);
		} else {
			String union = Character.toString(Initialization.ourSwitchPosition) + Character.toString(Initialization.scalePosition);
			if (union.equalsIgnoreCase("RL")) {
				if (Initialization.RLRStrat.equals("1")) {
					Autonomous.crossAutoline();
				} else if (Initialization.RLRStrat.equals("2")) {
					Autonomous.placeCubeOnSwitch();
				} else if (Initialization.RLRStrat.equals("3")) {
					Autonomous.placeCubeOnScale();
				}
			} else if (union.equalsIgnoreCase("LL")) {
				if (Initialization.LLLStrat.equals("1")) {
					Autonomous.crossAutoline();
				} else if (Initialization.LLLStrat.equals("2")) {
					Autonomous.placeCubeOnSwitch();
				} else if (Initialization.LLLStrat.equals("3")) {
					Autonomous.placeCubeOnScale();
				} else if (Initialization.LLLStrat.equals("4")) { // two cube
					
				} else if (Initialization.LLLStrat.equals("5")) { // three cube
					
				}
			} else if (union.equalsIgnoreCase("RR")) {
				if (Initialization.RRRStrat.equals("1")) {
					Autonomous.crossAutoline();
				} else if (Initialization.RRRStrat.equals("2")) {
					Autonomous.placeCubeOnSwitch();
				} else if (Initialization.RRRStrat.equals("3")) {
					Autonomous.placeCubeOnScale();
				} else if (Initialization.RRRStrat.equals("4")) { // two cube
					if(needToPopulateTwoCube) {
						RecordingList.populateTwoCube();	
						needToPopulateTwoCube = false;
					}
					RecordingList.setRobotValues(robotValuesI++);
				} else if (Initialization.RRRStrat.equals("5")) { // three cube
					if(needToPopulateThreeCube) {
						RecordingList.populateThreeCube();	
						needToPopulateThreeCube = false;
					}
					RecordingList.setRobotValues(robotValuesI++);
				}
			} else if (union.equalsIgnoreCase("LR")) {
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
		MatchData.log();
		/**/
	}

	public void teleopInit() {
		Initialization.gearaffesDrive.setSafetyEnabled(false);
		assignPreferenceVariables();
		//resetSensors();
		calibrateSensors();
		BuildingBlocks.setBraking(false);
		if (SmartDashboard.getBoolean("Record Robot", false)) {
			Autonomous.timeList.clear();
			Autonomous.BRList.clear();
			Autonomous.FRList.clear();
			Autonomous.BLList.clear();
			Autonomous.FLList.clear();
			Autonomous.elvList.clear();
			Autonomous.ejectPistonList.clear();
			Autonomous.gyroList.clear();
			Autonomous.encoderList.clear();
			Autonomous.differenceInGyroList.clear();
			Autonomous.differenceInEncoderList.clear();
			Autonomous.stepList.clear();
			Autonomous.summaryList.clear();
		}
		MatchData.beginLogging(Mode.TELEOP);
	}

	public void teleopPeriodic() {
		SmartDashboard.putNumber("grabber value", Initialization.grabberMotorController.get());
		Teleop.drive();
		Teleop.elevate();
		Teleop.ejectCube();
		Teleop.grabber();
		Teleop.climb();
		Teleop.endGameRumble();
		Teleop.rangeDistance();
		Teleop.elevatorRumble();
		if (SmartDashboard.getBoolean("Record Robot", false)) {
			Autonomous.record();
		}
		SmartDashboard.putNumber("Grabber Encoder Ticks", Initialization.grabberEncoder.getDistance());
		SmartDashboard.putNumber("GRABBO", Initialization.grabberMotorController.get());
		//NumberFormat.getNumberInstance().
		SmartDashboard.putNumber("Battery Consumed", formatValue(ChargeAccumulator.get()));
		MatchData.log();
	}

	public void testInit() {
		Initialization.gearaffesDrive.setSafetyEnabled(false);
		assignPreferenceVariables();
		resetSensors();
		calibrateSensors();
		Test.testSequenceIndex = 0;
		Test.determineTestSequence();
		Initialization.gearaffesPID = new GearaffesPID(Initialization.move_KP, Initialization.move_KI, Initialization.gyro, new GearaffesPID.GearaffesOutput());
		Initialization.gearaffesPID.enable();
		MatchData.beginLogging(Mode.TEST);
	}

	public void testPeriodic() {
		if (SmartDashboard.getBoolean("Run Test Sequence", false)) {
			Test.runTestSequence();
		}
		MatchData.log();
	}

	public void disabledInit() {
		MatchData.saveBulkLog();
		MatchData.saveLog();
		Initialization.gearaffesPID.reset();
		SmartDashboard.putBoolean("Record Robot", false);
		try {
		Autonomous.printer();
		} catch (IndexOutOfBoundsException ex ) {
			
		}
	}

	public void disabledPeriodic() {

	}

	public static void resetSensors() {
		Initialization.rightDriveEncoder.reset();
		Initialization.leftDriveEncoder.reset();
		Initialization.elevatorEncoder.reset();
		Initialization.gyro.reset();
		Initialization.grabberEncoder.reset();
	}

	public void calibrateSensors() {
		Initialization.elevatorEncoder.setDistancePerPulse(Initialization.ELEVATOR_INCHES_PER_TICK);
		Initialization.rightDriveEncoder.setDistancePerPulse(Initialization.DRIVE_INCHES_PER_TICK);
		Initialization.leftDriveEncoder.setDistancePerPulse(Initialization.DRIVE_INCHES_PER_TICK);
		Initialization.grabberEncoder.setDistancePerPulse(Initialization.GRABBER_DEGREES_PER_TICK);
		Initialization.elevatorEncoder.setSamplesToAverage(5);
		Initialization.rightDriveEncoder.setSamplesToAverage(4);
		Initialization.leftDriveEncoder.setSamplesToAverage(4);
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
		SmartDashboard.putNumber("Left Spedometer", Initialization.leftDriveEncoder.getRate());
		SmartDashboard.putNumber("Right Spedometer", Initialization.rightDriveEncoder.getRate());
		SmartDashboard.putNumber("Grabber Value", Initialization.grabberEncoder.getDistance());
	}

	public static void assignPreferenceVariables() {
		Initialization.autoStrat = Initialization.prefs.getString("Autonomous Strategy", "Not Found");

		// Autonomous Moving Speeds
		Initialization.autoMoveSpeed = Initialization.prefs.getDouble("Auto Move Speed", 70) / 100;
		Initialization.autoMoveContactHigh = Initialization.prefs.getDouble("Auto Move Contact High", 55) / 100;
		Initialization.autoMoveContactLow = Initialization.prefs.getDouble("Auto Move Contact Low", 30) / 100;
		Initialization.autoRotateSpeed = Initialization.prefs.getDouble("Auto Rotate Speed", 50) / 100;

		// Automation Moving Speeds
		Initialization.autoDelayTime = Initialization.prefs.getDouble("Autonomous Delay", 0);
		Initialization.automationHighSpeed = Initialization.prefs.getDouble("Automation High Speed", 70) / 100;
		Initialization.automationLowSpeed = Prefs.getDouble("Automation Low Speed", 30) / 100;
		Initialization.automationTopSpeed = Initialization.prefs.getDouble("Automation Top Speed", 40) / 100;
		Initialization.automationBottomSpeed = Initialization.prefs.getDouble("Automation Bottom Speed", 10) / 100;
		Initialization.automationElevatorHoldSpeed = Initialization.prefs.getDouble("Automation Hold Speed", 30) / 100;
		Initialization.automationGrabberHoldSpeed = Initialization.prefs.getDouble("Grabber Hold Speed", 0) / 100;
		Initialization.automationGrabberCubeHoldSpeed = Initialization.prefs.getDouble("Grabber Cube Hold Speed", 0) / 100;

		// Robot Starting Position
		determineAutoCode();

		// Teleop Multipliers
		Initialization.moveMultiplier = Initialization.prefs.getDouble("Move Multiplier", 70) / 100;
		Initialization.rotateMultiplier = Initialization.prefs.getDouble("Rotate Multiplier", 50) / 100;
		Initialization.elevateMultiplier = Initialization.prefs.getDouble("Elevate Multiplier", 70) / 100;
		Initialization.grabberMultiplier = Prefs.getDouble("Grabber Multiplier", 60) / 100;

		// PID Multipliers
		Initialization.move_KP = Initialization.prefs.getDouble("Move KP", 0.06);
		Initialization.move_KI = Initialization.prefs.getDouble("Move KI", 0.03);
		
		// Brake jawns
		Initialization.brakeB0 = Initialization.prefs.getDouble("Brake B0", 0);
		Initialization.brakeB1 = Initialization.prefs.getDouble("Brake B1", 0);
		Initialization.brakeKP = Initialization.prefs.getDouble("Brake KP", 0);
	}
	
	public static void determineAutoCode() {
		String[] autoCode = Initialization.prefs.getString("Autonomous Code", "!!!!!!").split("-");
		Initialization.robotStartingPosition = autoCode[0];
		Initialization.RLRStrat = autoCode[1];
		Initialization.RLRDelay = autoCode[2];
		Initialization.LLLStrat = autoCode[3];
		Initialization.LLLDelay = autoCode[4];
		Initialization.RRRStrat = autoCode[5];
		Initialization.RRRDelay = autoCode[6];
		Initialization.LRLStrat = autoCode[7];
		Initialization.LRLDelay = autoCode[8];
		
	}
	
}
