package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.Timer;

public class ChargeAccumulator {
	private static double chargeAmpSeconds;
	private static double chargeTotal = 0;
	private static double lastTime;
	public static void reset() {
		chargeAmpSeconds = 0;
		lastTime = -1;
	}
	public static double collectAndGet() { // returns total charge accumulated, in amp-hours
		double deltaTime;
		if(lastTime < 0) {
			deltaTime = 0;
		} else {
			deltaTime = Timer.getFPGATimestamp() - lastTime;
		}
		double fac = Initialization.pdp.getTotalCurrent() * deltaTime;
		chargeAmpSeconds += fac;
		chargeTotal += fac;
		lastTime = Timer.getFPGATimestamp();
		return chargeAmpSeconds / 3600;
	}
	public static double get() {
		return chargeTotal / 3600;
	}
}