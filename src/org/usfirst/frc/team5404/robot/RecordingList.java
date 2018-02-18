package org.usfirst.frc.team5404.robot;

import java.util.ArrayList;

public class RecordingList {
	public static ArrayList<Double> FRList = new ArrayList<Double>();
	public static ArrayList<Double> BRList = new ArrayList<Double>();
	public static ArrayList<Double> FLList = new ArrayList<Double>();
	public static ArrayList<Double> BLList = new ArrayList<Double>();
	public static ArrayList<Double> elvList = new ArrayList<Double>();
	public static ArrayList<Double> grabberList = new ArrayList<Double>();
	public static ArrayList<Boolean> ejectPistonList = new ArrayList<Boolean>();
	public static ArrayList<Boolean> grabberPistonList = new ArrayList<Boolean>();
		
	
	public static void setRobotValues(int i) {
		if(i<FRList.size()) {
			Initialization.FR.set(FRList.get(i));
			Initialization.BR.set(BRList.get(i));
			Initialization.FL.set(FLList.get(i));
			Initialization.BL.set(BLList.get(i));
			Initialization.elevator.set(elvList.get(i));
			Initialization.grabberMotorController.set(grabberList.get(i));
			Initialization.endEffectorPiston.set(ejectPistonList.get(i));
			Initialization.grabberPiston.set(grabberPistonList.get(i));
		}
		Initialization.FR.set(0);
		Initialization.BR.set(0);
		Initialization.FL.set(0);
		Initialization.BL.set(0);
		Initialization.elevator.set(0);
		Initialization.grabberMotorController.set(0);
		Initialization.endEffectorPiston.set(false);
		Initialization.grabberPiston.set(false);
	}
	
	
	public static void populateTwoCube() {
		FRList.clear();
		BRList.clear();
		FLList.clear();
		BLList.clear();
		elvList.clear();
		grabberList.clear();
		ejectPistonList.clear();
		grabberPistonList.clear();
		
		FRList.add(0.0);
		
		
		
		BRList.add(0.0);
		
		
		
		FLList.add(0.0);
		
		
		
		BLList.add(0.0);
		
		
		
		elvList.add(0.0);
		
		
		
		grabberList.add(0.0);
		
		
		
		ejectPistonList.add(false);
		
		
		
		grabberPistonList.add(false);
	}
	
	public static void populateThreeCube() {
		FRList.clear();
		BRList.clear();
		FLList.clear();
		BLList.clear();
		elvList.clear();
		grabberList.clear();
		ejectPistonList.clear();
		grabberPistonList.clear();
		
		FRList.add(0.0);
		
		
		
		BRList.add(0.0);
		
		
		
		FLList.add(0.0);
		
		
		
		BLList.add(0.0);
		
		
		
		elvList.add(0.0);
		
		
		
		grabberList.add(0.0);
		
		
		
		ejectPistonList.add(false);
		
		
		
		grabberPistonList.add(false);
	}
	

}
