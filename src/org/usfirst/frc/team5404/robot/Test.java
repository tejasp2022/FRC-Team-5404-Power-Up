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
		Initialization.controllerEncoderPairs.add(new SmartController("FL", Initialization.FL, Initialization.leftDriveEncoder));
		Initialization.controllerEncoderPairs.add(new SmartController("FR", Initialization.FR, Initialization.rightDriveEncoder));
		Initialization.controllerEncoderPairs.add(new SmartController("BL", Initialization.BL, Initialization.leftDriveEncoder));
		Initialization.controllerEncoderPairs.add(new SmartController("BR", Initialization.BR, Initialization.rightDriveEncoder));

		
		Initialization.testSequence = new ArrayList<>();
		Initialization.testSequence.add((Void) -> motorSeriesTest(Initialization.controllerEncoderPairs, 0.6, 5));
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
			diagnosticInfo.append(last.encoder.getDistance());
			startTime = -1;
			endTime = -1;
			lastMotorIndex = 0;
			System.out.println(diagnosticInfo.toString()); //Should output to Riolog
			return true;
		} else {
			int index = (int)Math.floor((endTime - startTime) / timePerMotor);
			if(index > lastMotorIndex) {
				SmartController last = controllers.get(lastMotorIndex);
				diagnosticInfo.append(last.name);
				diagnosticInfo.append(": ");
				diagnosticInfo.append(last.encoder.getDistance());
				diagnosticInfo.append("\r\n");
				controllers.get(index).encoder.reset();
			}
			SmartController sc = controllers.get(index);
			sc.controller.set(speed);
			lastMotorIndex = index;
			return false;
		}
	}
	public static class SmartController {
		public final SpeedController controller;
		public final Encoder encoder;
		public final String name;
		public SmartController(String name, SpeedController controller, Encoder encoder) {
			this.name = name;
			this.controller = controller;
			this.encoder = encoder;
		}
	}
}