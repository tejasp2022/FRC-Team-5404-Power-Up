package org.usfirst.frc.team5404.robot;

import java.io.File;
import java.io.FileWriter;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;

public class RecordingList {
	
	public static ArrayList<Double> leftMotorsList = new ArrayList<Double>();
	public static ArrayList<Double> leftEncoderList = new ArrayList<Double>();
	public static ArrayList<Double> rightMotorsList = new ArrayList<Double>();
	public static ArrayList<Double> rightEncoderList = new ArrayList<Double>();
	public static ArrayList<Double> grabberMotorList = new ArrayList<Double>();
	public static ArrayList<Double> grabberEncoderList = new ArrayList<Double>();
	public static ArrayList<Double> grabberPistonList = new ArrayList<Double>();
	public static ArrayList<Double> elevatorMotorList = new ArrayList<Double>();
	public static ArrayList<Double> elevatorEncoderList = new ArrayList<Double>();
	public static ArrayList<Double> elevatorPistonList = new ArrayList<Double>();
	public static ArrayList<Double> intakeMotorList = new ArrayList<Double>();
	public static ArrayList<Double> intakePistonList = new ArrayList<Double>();
	
	
	public static void populate() {
		leftMotorsList.add(Initialization.FL.get());
		leftEncoderList.add(Initialization.leftDriveEncoder.getDistance());
		rightMotorsList.add(Initialization.FR.get());
		rightEncoderList.add(Initialization.rightDriveEncoder.getDistance());
		grabberMotorList.add(Initialization.grabberMotorController.get());
		grabberEncoderList.add(Initialization.grabberEncoder.getDistance());
		grabberPistonList.add(Initialization.grabberPiston.get()? 1.0: 0.0);
		elevatorMotorList.add(Initialization.elevator.get());
		elevatorEncoderList.add(Initialization.elevatorEncoder.getDistance());
		elevatorPistonList.add(Initialization.endEffectorPiston.get()? 1.0: 0.0);
		intakeMotorList.add(0.0);
		intakePistonList.add(0.0);
	}
	
	public static void sendToCSV() {
        DateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd--HH-mm-ss");
        Date date = new Date();
        System.out.println(dateFormat.format(date)); 
        String fileName = "/media/sda1/Log--" + dateFormat.format(date)+ "Recording.csv";
        File file = new File(fileName);
        try {
            if (!file.exists()) {
                file.createNewFile();
            }
            FileWriter writer = new FileWriter(file);
            String entry = "Time, Left Motor, Left Motor Encoder, Right Motor, Right Motor Encoder, Grabber Motor, Grabber Encoder, Grabber Piston, Elevator Motor, Elevator Encoder, Elevator Piston, Intake Motor, Intake Piston\n";
            for (int i = 0; i < leftMotorsList.size(); i++) {
                double time = (i + 1) * 20;
                entry += time + ","
                        + leftMotorsList.get(i) + ","
                        + leftEncoderList.get(i) + ","
                        + rightMotorsList.get(i) + ","
                        + rightEncoderList.get(i) + ","
                        + grabberMotorList.get(i) + ","
                        + grabberEncoderList.get(i) + ","
                        + grabberPistonList.get(i) + ","
                        + elevatorMotorList.get(i) + ","
                        + elevatorEncoderList.get(i) + ","
                        + elevatorPistonList.get(i) + ","
                        + intakeMotorList.get(i) + ","
                        + intakePistonList.get(i) + "\n";
            }
            writer.write(entry);
            writer.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}