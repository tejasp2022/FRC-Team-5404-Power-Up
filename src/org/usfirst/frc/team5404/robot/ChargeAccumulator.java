package org.usfirst.frc.team5404.robot;

public class ChargeAccumulator {
	private static double chargeAmpSeconds;
	public static void reset() {
		chargeAmpSeconds = 0;
	}
	public static double collectAndGet() { // returns total charge accumulated, in amp-hours
		chargeAmpSeconds += Initialization.pdp.getTotalCurrent() * 0.020;
		return chargeAmpSeconds / 3600;
	}
}