package org.usfirst.frc.team5404.robot;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Test {
	public static void determineTestSequence() {
		Initialization.controllerEncoderPairs = new ArrayList<>();
		Initialization.controllerEncoderPairs.add(new SmartController("FL", Initialization.FL, Initialization.leftDriveEncoder, false));
		Initialization.controllerEncoderPairs.add(new SmartController("FR", Initialization.FR, Initialization.rightDriveEncoder, true));
		Initialization.controllerEncoderPairs.add(new SmartController("BL", Initialization.BL, Initialization.leftDriveEncoder, false));
		Initialization.controllerEncoderPairs.add(new SmartController("BR", Initialization.BR, Initialization.rightDriveEncoder, true));
		Initialization.testSequence = new ArrayList<>();
		if (SmartDashboard.getBoolean("Motor Series Test", false)) {
			Initialization.testSequence.add((Void) -> {
				DriveBase.setBraking(Prefs.getBoolean("Test Brake On", false));
				return true;
			});
			Initialization.testSequence.add((Void) -> BuildingBlocks.delay(1));
			Initialization.testSequence.add((Void) -> motorSeriesTest(Initialization.controllerEncoderPairs, Prefs.getDouble("Test Power", 0.6), 5));
			Initialization.testSequence.add((Void) -> {
				DriveBase.setBraking(false);
				return true;
			});
		} else if (SmartDashboard.getBoolean("Motor Individual Test", false)) {
			Initialization.testSequence.add((Void) -> motorIndividualTest(Initialization.controllerEncoderPairs.get(3), 5));
		} else if (SmartDashboard.getBoolean("Elevator Speed Range Test", false)) {
			SmartController elevator = new SmartController("Elevator", Initialization.elevator, Initialization.elevatorEncoder, false);
			Initialization.testSequence.add((Void) -> elevatorSpeedRangeTest(elevator, true));
			Initialization.testSequence.add((Void) -> bringElevatorToTop(elevator));
			Initialization.testSequence.add((Void) -> elevatorSpeedRangeTest(elevator, false));
		} else if(SmartDashboard.getBoolean("Motor Brake Test", false)) {
			double distance = Prefs.getDouble("Test Drive Distance", 0);
			double speed = Prefs.getDouble("Test Drive Speed", 0.5);
			//double Kp = Prefs.getDouble("Brake KP", 0.006);
			Initialization.testSequence.add((Void) -> brakeTest(Initialization.gearaffesDrive, distance, speed));
		} else if(SmartDashboard.getBoolean("Rotate Brake Test", false)) {
			double angle = Prefs.getDouble("Test Drive Angle", 90);
			double speed = Prefs.getDouble("Test Drive Speed", 0.5);
			Initialization.testSequence.add((Void)-> rotateBrakeTest(Initialization.gearaffesDrive, angle, speed));
		} else if(SmartDashboard.getBoolean("Multiple Rotation Test", false)) {
			Average.reset();
			Average.newAverage("speed");
			Average.newAverage("angle");
			for(int i = 0; i < 30; i++) {
				Initialization.testSequence.add((Void) -> rotateBrakeTest(Initialization.gearaffesDrive, Prefs.getDouble("Test Drive Angle", 90),
						Prefs.getDouble("Test Drive Speed", 0.9)));
				Initialization.testSequence.add((Void) -> BuildingBlocks.delay(1.5));
			}
			Initialization.testSequence.add((Void) -> {
				System.out.println("AVGSpeed: " + Average.get("speed"));
				System.out.println("AVGAngle: " + Average.get("angle"));
				return true;
			});
		}
		principalV = Initialization.pdp.getVoltage();
	}

	public static int testSequenceIndex;

	public static void runTestSequence() {
		if (testSequenceIndex < Initialization.testSequence.size()) {
			if (Initialization.testSequence.get(testSequenceIndex).apply(null)) {
				Robot.resetSensors();
				testSequenceIndex++;
			}
		} else {
			SmartDashboard.putString("Test Sequence", "Finished test sequence.");
			SmartDashboard.putBoolean("Run Test Sequence", false);
			SmartDashboard.putBoolean("Motor Series Test", false);
			SmartDashboard.putBoolean("Motor Individual Test", false);
			SmartDashboard.putBoolean("Elevator Speed Range Test", false);
			SmartDashboard.putBoolean("Motor Brake Test", false);
			SmartDashboard.putBoolean("Rotate Brake Test", false);
		}
	}

	public static double startTime = -1, endTime = -1;
	public static int lastMotorIndex;
	public static StringBuilder diagnosticInfo;
	public static double principalV;
	public static final double ELEVATOR_MAX = 72;
	public static double upDrive = 0;
	public static boolean hasFallen = false;
	public static int successes = 0;
	public static boolean isBraking = false;
	public static double leftStartedBrake, rightStartedBrake, gyroStartedBrake;
	
	public static boolean brakeTest(DifferentialDrive drive, double dist, double speed) {
		if(isBraking) {
			double leftRate = Initialization.leftDriveEncoder.getRate();
			double rightRate = Initialization.rightDriveEncoder.getRate();
			drive.arcadeDrive(0, 0);
			DriveBase.setBraking(true);
			if(Math.abs(leftRate) < 0.8 && Math.abs(rightRate) < 0.8) {
				double lbd = Math.abs(Initialization.leftDriveEncoder.getDistance() - leftStartedBrake);
				double rbd = Math.abs(Initialization.rightDriveEncoder.getDistance() - rightStartedBrake);
				System.out.println("Left Brake Distance: " + lbd);
				System.out.println("Right Brake Distance: " + rbd);
				System.out.println("AVG DIST: " + ((lbd + rbd) / 2));
				isBraking = false;
				drive.arcadeDrive(0, 0);
				return true;
			} else {
				return false;
			}
		} else { 
			if (Math.abs(Initialization.leftDriveEncoder.getDistance()) < (Math.abs(dist))) {
				if (dist > 0) {
					drive.arcadeDrive(speed, Initialization.gearaffesPID.get(), false);																					
				} else {
					drive.arcadeDrive(-speed, Initialization.gearaffesPID.get(), false);
				}
				Robot.displaySensors();
				return false;
			} else {
				leftStartedBrake = Initialization.leftDriveEncoder.getDistance();
				rightStartedBrake = Initialization.rightDriveEncoder.getDistance();
				double lRate = Math.abs(Initialization.leftDriveEncoder.getRate());
				double rRate = Math.abs(Initialization.rightDriveEncoder.getRate());
				System.out.println("Left Rate: " + lRate);
				System.out.println("Right Rate: " + rRate);
				System.out.println("AVG RATE: " + ((lRate + rRate) / 2));
				drive.arcadeDrive(0, 0);
				isBraking = true;
				return false;
			}
		}
	}
	public static boolean rotateBrakeTest(DifferentialDrive drive, double angle, double speed) {
		if(isBraking) {
			double rate = Initialization.gyro.getRate();
			drive.arcadeDrive(0, 0);
			DriveBase.setBraking(true);
			if(Math.abs(rate) < 0.3) {
				double gAngle = Initialization.gyro.getAngle();
				double lbd = Math.abs(gAngle - gyroStartedBrake);
				System.out.println("Input Angle: " + angle);
				System.out.println("Braked At: " + gyroStartedBrake);
				System.out.println("Stopped At: " + gAngle);
				System.out.println("Gyro Brake Angle: " + lbd);
				Average.add("angle", gAngle - angle);
				isBraking = false;
				drive.arcadeDrive(0, 0);
				DriveBase.setBraking(false);
				return true;
			} else {
				return false;
			}
		} else { 
			if (Math.abs(Initialization.gyro.getAngle()) < (Math.abs(angle))) {
				if (angle > 0) {
					drive.arcadeDrive(0, speed, false);																					
				} else {
					drive.arcadeDrive(0, -speed, false);
				}
				Robot.displaySensors();
				return false;
			} else {
				gyroStartedBrake = Initialization.gyro.getAngle();
				double gRate = Math.abs(Initialization.gyro.getRate());
				System.out.println("Spin Rate: " + gRate);
				Average.add("speed", gRate);
				drive.arcadeDrive(0, 0);
				isBraking = true;
				return false;
			}
		}
	}

	public static boolean elevatorSpeedRangeTest(SmartController controller, boolean up) {
		if (up) {
			if (Math.abs(controller.encoder.getRate()) < 3) {
				upDrive += 0.001;
				controller.controller.set(upDrive);
			} else {
				successes++;
			}
			if (successes >= 5) {
				System.out.println("Drive needed to move up: " + upDrive);
				upDrive = 0;
				hasFallen = false;
				successes = 0;
				return true;
			}
		} else {
			if (startTime == -1) {
				startTime = Timer.getFPGATimestamp();
			}
			if (!hasFallen) {
				if ((Timer.getFPGATimestamp() - startTime) % 1 > 0.5) {
					controller.controller.set(0);
					if (Math.abs(controller.encoder.getRate()) > 3) {
						successes++;
					}
					if (successes >= 5) {
						hasFallen = true;
						successes = 0;
					}
				} else {
					controller.controller.set(-0.4);
					successes = 0;
				}
			} else {
				controller.controller.set(upDrive);
				if (Math.abs(controller.encoder.getRate()) > 3) {
					upDrive += 0.01;
				} else {
					System.out.println("Drive needed to hold: " + upDrive);
					upDrive = 0;
					hasFallen = false;
					successes = 0;
					return true;
				}
			}
		}
		return false;
	}

	public static boolean bringElevatorToTop(SmartController controller) {
		if (controller.encoder.getDistance() > ELEVATOR_MAX) {
			controller.controller.set(0);
			return true;
		}
		controller.controller.set(Initialization.automationHighSpeed);
		return false;
	}

	public static boolean motorSeriesTest(List<SmartController> controllers, double speed, double timePerMotor) {
		if (startTime == -1) {
			Average.reset();
			Average.newAverage("current");
			Average.newAverage("voltage");
			startTime = Timer.getFPGATimestamp();
			endTime = startTime + controllers.size() * timePerMotor;
			diagnosticInfo = new StringBuilder();
			lastMotorIndex = 0;
		}
		if (endTime <= Timer.getFPGATimestamp()) {
			SmartController last = controllers.get(lastMotorIndex);
			double d = last.invertFactor * last.encoder.getDistance();
			double I = Average.get("current");
			double V = Average.get("voltage");
			diagnosticInfo.append(last.name);
			diagnosticInfo.append("\r\n\tTotal encoder distance: ");
			diagnosticInfo.append(d);
			diagnosticInfo.append("\"\r\n\tAverage current: ");
			diagnosticInfo.append(I);
			diagnosticInfo.append("A\r\n\tAverage voltage drop: ");
			diagnosticInfo.append(V);
			diagnosticInfo.append("V\r\n\tConstant: ");
			diagnosticInfo.append(d / I / V / timePerMotor);
			diagnosticInfo.append("in/J");
			Average.reset();
			last.controller.set(0);
			startTime = -1;
			endTime = -1;
			lastMotorIndex = 0;
			System.out.println(diagnosticInfo.toString()); // Should output to Riolog
			return true;
		} else {
			int index = (int) Math.floor((Timer.getFPGATimestamp() - startTime) / timePerMotor);
			if (index > lastMotorIndex) {
				SmartController last = controllers.get(lastMotorIndex);
				System.out.println("INPERSEC: " + last.encoder.getRate());
				double d = last.invertFactor * last.encoder.getDistance();
				double I = Average.get("current");
				double V = Average.get("voltage");
				diagnosticInfo.append(last.name);
				diagnosticInfo.append("\r\n\tTotal encoder distance: ");
				diagnosticInfo.append(d);
				diagnosticInfo.append("\"\r\n\tAverage current: ");
				diagnosticInfo.append(I);
				diagnosticInfo.append("A\r\n\tAverage voltage drop: ");
				diagnosticInfo.append(V);
				diagnosticInfo.append("V\r\n\tConstant: ");
				diagnosticInfo.append(d / I / V / timePerMotor);
				diagnosticInfo.append("in/J\r\n");
				Average.reset();
				Average.newAverage("current");
				Average.newAverage("voltage");
				last.controller.set(0);
				controllers.get(index).encoder.reset();
			}
			SmartController sc = controllers.get(index);
			sc.controller.set(sc.invertFactor * speed);
			Average.add("current", Initialization.pdp.getTotalCurrent());
			Average.add("voltage", principalV - Initialization.pdp.getVoltage());
			lastMotorIndex = index;
			return false;
		}
	}

	public static boolean motorIndividualTest(SmartController controller, double timePerSpeed) {
		// speeds: 0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1
		if (startTime == -1) {
			Average.newAverage("current");
			Average.newAverage("voltage");
			startTime = Timer.getFPGATimestamp();
			endTime = startTime + 11 * (timePerSpeed + 1);
			diagnosticInfo = new StringBuilder();
			lastMotorIndex = 0;
		}
		if (endTime <= Timer.getFPGATimestamp()) {
			double d = controller.invertFactor * controller.encoder.getDistance();
			double I = Average.get("current");
			double V = Average.get("voltage");
			diagnosticInfo.append(controller.name + " at " + lastMotorIndex * 0.1);
			diagnosticInfo.append("\r\n\tTotal encoder distance: ");
			diagnosticInfo.append(d);
			diagnosticInfo.append("\"\r\n\tAverage current: ");
			diagnosticInfo.append(I);
			diagnosticInfo.append("A\r\n\tAverage voltage drop: ");
			diagnosticInfo.append(V);
			Average.reset();
			controller.controller.set(0);
			startTime = -1;
			endTime = -1;
			lastMotorIndex = 0;
			System.out.println(diagnosticInfo.toString()); // Should output to Riolog
			return true;
		} else {
			int index = (int) Math.floor((Timer.getFPGATimestamp() - startTime) / (timePerSpeed + 1));
			double speed = Math.min(0.1 * index, 1);
			if (index > lastMotorIndex) {
				double d = controller.invertFactor * controller.encoder.getDistance();
				double I = Average.get("current");
				double V = Average.get("voltage");
				diagnosticInfo.append(controller.name + " at " + lastMotorIndex * 0.1);
				diagnosticInfo.append("\r\n\tTotal encoder distance: ");
				diagnosticInfo.append(d);
				diagnosticInfo.append("\"\r\n\tAverage current: ");
				diagnosticInfo.append(I);
				diagnosticInfo.append("A\r\n\tAverage voltage drop: ");
				diagnosticInfo.append(V);
				diagnosticInfo.append("\r\n");
				Average.reset();
				Average.newAverage("current");
				Average.newAverage("voltage");
				controller.controller.set(0);
				controller.encoder.reset();
			} else {
				if (((Timer.getFPGATimestamp() - startTime) - (index * (timePerSpeed + 1))) > timePerSpeed) {
					controller.controller.set(0);
				} else {
					controller.controller.set(controller.invertFactor * speed);
					Average.add("current", Initialization.pdp.getTotalCurrent());
					Average.add("voltage", principalV - Initialization.pdp.getVoltage());
				}
			}
			lastMotorIndex = index;
			return false;
		}
	}

	public static class SmartController {
		public final SpeedController controller;
		public final Encoder encoder;
		public final String name;
		public final double invertFactor;

		public SmartController(String name, SpeedController controller, Encoder encoder, boolean invert) {
			this.name = name;
			this.controller = controller;
			this.encoder = encoder;
			if (invert) {
				this.invertFactor = -1;
			} else {
				this.invertFactor = 1;
			}
		}
	}

	public static class Average {
		public static HashMap<String, List<Double>> data;

		public static void reset() {
			data = new HashMap<>();
		}

		public static void newAverage(String name) {
			data.put(name, new ArrayList<Double>());
		}

		public static void add(String name, double d) {
			data.get(name).add(d);
		}

		public static double get(String name) {
			double sum = 0;
			List<Double> toAvg = data.get(name);
			for (double a : toAvg) {
				sum += a;
			}
			return sum / toAvg.size();
		}
	}
}