package org.usfirst.frc.team5404.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Test {
	public static void determineTestSequence() {
		Initialization.controllerEncoderPairs = new ArrayList<>();
		Initialization.controllerEncoderPairs.add(new SmartController("FL", Initialization.FL, Initialization.leftDriveEncoder, false));
		Initialization.controllerEncoderPairs.add(new SmartController("FR", Initialization.FR, Initialization.rightDriveEncoder, true));
		Initialization.controllerEncoderPairs.add(new SmartController("BL", Initialization.BL, Initialization.leftDriveEncoder, false));
		Initialization.controllerEncoderPairs.add(new SmartController("BR", Initialization.BR, Initialization.rightDriveEncoder, true));
		toAverage = new ArrayList<>();
		Initialization.testSequence = new ArrayList<>();
		Initialization.testSequence.add((Void) -> motorSeriesTest(Initialization.controllerEncoderPairs, Initialization.prefs.getDouble("Test Power", 0.6), 5));
	}
	public static int testSequenceIndex;
	public static void runTestSequence() {
		if(testSequenceIndex < Initialization.testSequence.size()) {
			if(Initialization.testSequence.get(testSequenceIndex).apply(null)) {
				Robot.resetSensors();
				testSequenceIndex++;
			}
		} else {
			SmartDashboard.putString("Test Sequence", "Finished test sequence.");
		}
	}
	public static double startTime = -1, endTime = -1;
	public static int lastMotorIndex;
	public static StringBuilder diagnosticInfo;
	public static List<Double> toAverage;
	public static boolean motorSeriesTest(List<SmartController> controllers, double speed, double timePerMotor) {
		if(startTime == -1) {
			startTime = Timer.getFPGATimestamp();
			endTime = startTime + controllers.size() * timePerMotor;
			diagnosticInfo = new StringBuilder();
			lastMotorIndex = 0;
		}
		if(endTime <= Timer.getFPGATimestamp()) {
			SmartController last = controllers.get(lastMotorIndex);
			diagnosticInfo.append(last.name);
			diagnosticInfo.append(": ");
			diagnosticInfo.append(last.invertFactor * last.encoder.getDistance());
			diagnosticInfo.append("; ");
			diagnosticInfo.append(avg());
			toAverage.clear();
			last.controller.set(0);
			startTime = -1;
			endTime = -1;
			lastMotorIndex = 0;
			System.out.println(diagnosticInfo.toString()); //Should output to Riolog
			return true;
		} else {
			int index = (int)Math.floor((Timer.getFPGATimestamp() - startTime) / timePerMotor);
			if(index > lastMotorIndex) {
				SmartController last = controllers.get(lastMotorIndex);
				diagnosticInfo.append(last.name);
				diagnosticInfo.append(": ");
				diagnosticInfo.append(last.invertFactor * last.encoder.getDistance());
				diagnosticInfo.append("; ");
				diagnosticInfo.append(avg());
				diagnosticInfo.append("\r\n");
				toAverage.clear();
				last.controller.set(0);
				controllers.get(index).encoder.reset();
			}
			SmartController sc = controllers.get(index);
			sc.controller.set(sc.invertFactor * speed);
			toAverage.add(Initialization.pdp.getTotalCurrent());
			lastMotorIndex = index;
			return false;
		}
	}
	public static double avg() {
		double add = 0;
		for(double a : toAverage) {
			add += a;
		}
		return add / toAverage.size();
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
			if(invert) {
				this.invertFactor = -1;
			} else {
				this.invertFactor = 1;
			}
		}
	}
}