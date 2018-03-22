package org.usfirst.frc.team5404.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team5404.robot.GearaffesPID.GearaffesOutput;

public class Playback {
	private final GearaffesPID leftEncoderPID, rightEncoderPID;
	private final double[][] playbackData;
	private int playbackIndex = 0;
	private Playback(double[][] playbackData) {
		this.playbackData = playbackData;
		leftEncoderPID = new GearaffesPID(Prefs.getDouble("Playback Drive KP", 0.0001),
				Prefs.getDouble("Playback Drive KI", 0), Initialization.leftDriveEncoder, new GearaffesOutput());
		rightEncoderPID = new GearaffesPID(Prefs.getDouble("Playback Drive KP", 0.0001),
				Prefs.getDouble("Playback Drive KI", 0), Initialization.rightDriveEncoder, new GearaffesOutput());
	}
	private static double[] parseDoubles(String[] values) {
		double[] parsed = new double[values.length];
		for(int i = 0; i < parsed.length; i++) {
			parsed[i] = Double.valueOf(values[i]);
		}
		return parsed;
	}
	public static Playback loadFromFile(File input) {
		if(!input.exists()) {
			new FileNotFoundException(input.getPath()).printStackTrace();
			return null;
		}
		try {
			BufferedReader reader = new BufferedReader(new FileReader(input));
			String line;
			List<String> lines = new ArrayList<String>();
			while((line = reader.readLine()) != null) {
				lines.add(line);
			}
			reader.close();
			double[][] playbackData = new double[lines.size()][];
			for(int i = 1; i < lines.size(); i++) { // i=0 is headers
				line = lines.get(i);
				double[] inputs = parseDoubles(line.split(","));
				playbackData[i] = inputs;
			}
			return new Playback(playbackData);
		} catch(IOException e) {
			e.printStackTrace();
			return null;
		}
	}
	public void beginPlayback() {
		leftEncoderPID.reset();
		leftEncoderPID.enable();
		rightEncoderPID.reset();
		rightEncoderPID.enable();
		playbackIndex = 0;
	}
	private static double constrain(double m) {
		return Math.min(Math.max(-1, m), 1);
	}
	private static boolean bool(double m) {
		return m >= .5;
	}
	public void playbackPeriodic() {
		// values
		double[] vals = playbackData[playbackIndex];
		double L_m_val = vals[1]; //vals[0] = time
		double L_e_val = vals[2];
		double R_m_val = vals[3];
		double R_e_val = vals[4];
		double G_m_val = vals[5];
		double G_e_val = vals[6];
		double G_p_val = vals[7];
		double E_m_val = vals[8];
		double E_e_val = vals[9];
		double E_p_val = vals[10];
		double I_m_val = vals[11];
		double I_p_val = vals[12];
		
		// PIDs
		leftEncoderPID.setSetpoint(L_e_val);
		rightEncoderPID.setSetpoint(R_e_val);
		
		// Setting motor values
		double L_out = constrain(L_m_val + leftEncoderPID.get());
		double R_out = constrain(R_m_val + rightEncoderPID.get());
		Initialization.BL.set(L_out);
		Initialization.FL.set(L_out);
		Initialization.BR.set(R_out);
		Initialization.FR.set(R_out);
		Initialization.grabberMotorController.set(constrain(G_m_val)); // grabber encoder not used because really who cares right now
		Initialization.grabberPiston.set(bool(G_p_val));
		Initialization.elevator.set(E_m_val);
		Initialization.endEffectorPiston.set(bool(E_p_val));
		
		
		
		
		playbackIndex++;
	}
}