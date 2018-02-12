package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class GearaffesPID extends PIDController {

	public GearaffesOutput gOutput;
	public GearaffesPID(double Kp, double Ki, PIDSource source, GearaffesOutput output) {
		super(Kp, Ki, 0, source, output);
		gOutput = output;
	}
	
	public static class GearaffesOutput implements PIDOutput {
		public GearaffesOutput() { }
		public double output;
		public void pidWrite(double output) {
			this.output = output;
		}
	}
}