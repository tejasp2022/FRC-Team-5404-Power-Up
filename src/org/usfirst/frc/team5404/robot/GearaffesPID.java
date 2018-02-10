package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class GearaffesPID extends PIDController {

	public GearaffesOutput gOutput;
	public GearaffesGyroSource gSource;
	public GearaffesPID(double Kp, double Ki, GearaffesGyroSource source, GearaffesOutput output) {
		super(Kp, Ki, 0, source, output);
		gOutput = output;
		gSource = source;
	}
	
	public static class GearaffesOutput implements PIDOutput {
		public GearaffesOutput() { }
		public double output;
		public void pidWrite(double output) {
			this.output = output;
		}
	}
	public static class GearaffesGyroSource implements PIDSource {
		
		private ADXRS450_Gyro gyro;
		private double baseAngle;
		public GearaffesGyroSource(ADXRS450_Gyro gyro) {
			this.gyro = gyro;
		}
		public void setBaseAngle(double baseAngle) {
			this.baseAngle = baseAngle;
		}
		public double getAngle() {
			return gyro.getAngle() - baseAngle;
		}
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			gyro.setPIDSourceType(pidSource);
		}
		@Override
		public PIDSourceType getPIDSourceType() {
			return gyro.getPIDSourceType();
		}
		@Override
		public double pidGet() {
			return getAngle();
		}
	}
}