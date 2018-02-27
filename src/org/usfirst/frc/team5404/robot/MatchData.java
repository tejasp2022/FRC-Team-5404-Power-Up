package org.usfirst.frc.team5404.robot;

import java.io.File;
import java.io.FileWriter;
import java.io.Serializable;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;

import org.usfirst.frc.team5404.robot.Test.Average;

import edu.wpi.first.wpilibj.Timer;

public class MatchData {
	private static HashMap<String, Function<Void, Double>> matchCollectors;
	private static List<String> indices;
	private static List<MatchDataFrame> dataLog;
	private static long beginLogAt = -1;
	private static double timeStampBegin = -1;
	private static boolean isLogging = false;
	private static Mode logMode;
	public static void beginLogging(Mode mode) {
		logMode = mode;
		if(!isLogging) {
			checkAndAddComponents();
			dataLog = new ArrayList<MatchDataFrame>();
			beginLogAt = System.currentTimeMillis();
			timeStampBegin = Timer.getFPGATimestamp();
			isLogging = true;
		}
	}
	public static void log() {
		MatchDataFrame frame = new MatchDataFrame(Timer.getFPGATimestamp() - timeStampBegin);
		Set<Map.Entry<String, Function<Void, Double>>> entrySet = matchCollectors.entrySet();
		for(Map.Entry<String, Function<Void, Double>> entry : entrySet) {
			String id = entry.getKey();
			double val = entry.getValue().apply(null);
			frame.putData(id, val);
		}
		dataLog.add(frame);
	}
	public static void saveBulkLog() {
		if(beginLogAt > 0) {
			DateFormat df = new SimpleDateFormat("yyyy-MM-dd--HH-mm-ss");
			String fileName = "/media/sda1/Log--" + df.format(new Date(beginLogAt)) + logMode.toString() + "BULK.csv";
			File file = new File(fileName);
			try {
				if(!file.exists()) {
					file.createNewFile();
				}
				FileWriter writer = new FileWriter(file);
				StringBuilder builder = new StringBuilder();
				builder.append("time,");
				for(int i = 1; i < indices.size(); i++) {
					String key = indices.get(i);
					builder.append(key);
					builder.append(',');
				}
				builder.append("\r\n");
				writer.write(builder.toString());
				for(int j = 0; j < dataLog.size(); j++) {
					writer.write(dataLog.get(j).toString(indices));
				}
				writer.flush();
				writer.close();
			} catch(Exception e) {
				e.printStackTrace();
			}
		}
	}
	public static void saveLog() {
		if(beginLogAt > 0) {
			isLogging = false;
			DateFormat df = new SimpleDateFormat("yyyy-MM-dd--HH-mm-ss");
			String fileName = "/media/sda1/Log--" + df.format(new Date(beginLogAt)) + logMode.toString() + ".csv";
			File file = new File(fileName);
			try {
				List<List<MatchDataFrame>> allFrames = dataToMinMaxMean(dataLog, 25);
				if(!file.exists()) {
					file.createNewFile();
				}
				FileWriter writer = new FileWriter(file);
				StringBuilder builder = new StringBuilder();
				builder.append("time,");
				for(int i = 1; i < indices.size(); i++) {
					String key = indices.get(i);
					builder.append("MIN ");
					builder.append(key);
					builder.append(',');
					builder.append("MAX ");
					builder.append(key);
					builder.append(',');
					builder.append("MEAN ");
					builder.append(key);
					builder.append(',');
				}
				builder.append("\r\n");
				writer.write(builder.toString());
				List<MatchDataFrame> mean = allFrames.get(2);
				List<MatchDataFrame> min = allFrames.get(0);
				List<MatchDataFrame> max = allFrames.get(1);
				for(int j = 0; j < mean.size(); j++) {
					writer.write(MatchDataFrame.toString(indices, min.get(j), max.get(j), mean.get(j)));
					//writer.write(dataLog.get(j).toString(indices));
				}
				writer.flush();
				writer.close();
			} catch(Exception e) {
				e.printStackTrace();
			}
		}
	}
	private static List<List<MatchDataFrame>> dataToMinMaxMean(List<MatchDataFrame> frames, int samplesToAverage) {
		List<List<MatchDataFrame>> matchData = new ArrayList<>();
		List<MatchDataFrame> min = new ArrayList<>();
		List<MatchDataFrame> max = new ArrayList<>();
		List<MatchDataFrame> mean = new ArrayList<>();
		double time = 0;
		List<Double> mins = new ArrayList<>();
		List<Double> maxes = new ArrayList<>();
		for(int i = 0; i < frames.size(); i++) {
			MatchDataFrame frame = frames.get(i);
			if(i % samplesToAverage == 0) {
				Average.reset();
				mins = new ArrayList<>();
				maxes = new ArrayList<>();
				for(int j = 1; j < indices.size(); j++) {
					Average.newAverage(indices.get(j));
					mins.add(frame.getData(indices.get(j)));
					maxes.add(frame.getData(indices.get(j)));
				}
				time = frames.get(i).time;
			}
			for(int j = 1; j < indices.size(); j++) {
				Test.Average.add(indices.get(j), frame.getData(indices.get(j)));
				mins.set(j-1, Math.min(mins.get(j-1), frame.getData(indices.get(j))));
				maxes.set(j-1, Math.max(maxes.get(j-1), frame.getData(indices.get(j))));
			}
			if(i % samplesToAverage == samplesToAverage - 1) {
				// last frame
				MatchDataFrame frameMean = new MatchDataFrame(time);
				MatchDataFrame frameMin = new MatchDataFrame(time);
				MatchDataFrame frameMax = new MatchDataFrame(time);
				for(int j = 1; j < indices.size(); j++) {
					frameMean.putData(indices.get(j), Average.get(indices.get(j)));
					frameMin.putData(indices.get(j), mins.get(j-1));
					frameMax.putData(indices.get(j), maxes.get(j-1));
				}
				min.add(frameMin);
				max.add(frameMax);
				mean.add(frameMean);
			}
		}
		
		matchData.add(min);
		matchData.add(max);
		matchData.add(mean);
		return matchData;
	}
	/*
	private static double booleanToDouble(Function<Void, Boolean> function) {
		return function.apply(null) ? 1 : 0;
	}*/
	private static void checkAndAddComponents() {
		matchCollectors = new HashMap<>();
		indices = new ArrayList<>();
		indices.add("time");
		ChargeAccumulator.reset();
		checkAndAddComponent("Left Encoder Distance", (Void) -> Initialization.leftDriveEncoder.getDistance());
		checkAndAddComponent("Right Encoder Distance", (Void) -> -Initialization.rightDriveEncoder.getDistance());
		checkAndAddComponent("Left Encoder Rate", (Void) -> Initialization.leftDriveEncoder.getRate());
		checkAndAddComponent("Right Encoder Rate", (Void) -> -Initialization.rightDriveEncoder.getRate());
		checkAndAddComponent("Gyro Angle", (Void) -> Initialization.gyro.getAngle());
		checkAndAddComponent("Gyro Rate", (Void) -> Initialization.gyro.getRate());
		checkAndAddComponent("Charge Used", (Void) -> ChargeAccumulator.collectAndGet());
		checkAndAddComponent("PDP Voltage", (Void) -> Initialization.pdp.getVoltage());
		checkAndAddComponent("PDP Total Current", (Void) -> Initialization.pdp.getTotalCurrent());
		checkAndAddComponent("PDP Temperature", (Void) -> Initialization.pdp.getTemperature());
		for(int i = 0; i < 16; i++) {
			final int k = i;
			checkAndAddComponent("PDP Ch" + i, (Void) -> Initialization.pdp.getCurrent(k));
		}
		//checkAndAddComponent("Compressor Active", (Void) -> booleanToDouble((Void) -> Initialization))
	}
	private static boolean checkAndAddComponent(String id, Function<Void, Double> function) {
		try {
			function.apply(null);
			matchCollectors.put(id, function);
			indices.add(id);
			System.out.println("Added: " + id);
			return true;
		} catch(Exception e) {
			System.out.println("Failed to add component: " + id);
			e.printStackTrace();
			return false;
		}
	}
	public static class MatchDataFrame implements Serializable {
		private static final long serialVersionUID = -408130369712900978L;
		private final double time;
		private final HashMap<String, Double> data;
		public MatchDataFrame(double time) {
			this.time = time;
			data = new HashMap<>();
		}
		public void putData(String id, Double d) {
			data.put(id, d);
		}
		public double getData(String id) {
			return data.get(id);
		}
		@Override @Deprecated public String toString() {
			StringBuilder builder = new StringBuilder();
			Set<Map.Entry<String, Double>> datums = data.entrySet();
			builder.append("time,");
			builder.append(time);
			builder.append(";");
			for(Map.Entry<String, Double> datum : datums) {
				builder.append(datum.getKey());
				builder.append(',');
				builder.append(datum.getValue());
				builder.append(";");
			}
			builder.append("\r\n");
			return builder.toString();
		}
		public String toString(List<String> indices) {
			StringBuilder builder = new StringBuilder();
			for(int i = 0; i < indices.size(); i++) {
				String key = indices.get(i);
				String val = String.valueOf(data.get(key));
				builder.append(val);
				builder.append(',');
			}
			builder.append("\r\n");
			return builder.toString();
		}
		public static String toString(List<String> indices, MatchDataFrame min, MatchDataFrame max, MatchDataFrame mean) {
			StringBuilder builder = new StringBuilder();
			builder.append(mean.time);
			builder.append(",");
			for(int i = 1; i < indices.size(); i++) {
				builder.append(min.getData(indices.get(i)));
				builder.append(",");
				builder.append(max.getData(indices.get(i)));
				builder.append(",");
				builder.append(mean.getData(indices.get(i)));
				builder.append(",");
			}
			builder.append("\r\n");
			return builder.toString();
		}
	}
	/*public static void main(String[] args) {
		MatchData.beginLogging(Mode.AUTO);
	}*/
}