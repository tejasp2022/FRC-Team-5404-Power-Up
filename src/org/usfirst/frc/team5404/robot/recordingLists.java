package org.usfirst.frc.team5404.robot;

import java.util.ArrayList;

public class recordingLists {
	public static ArrayList<Double> twoCubeFR = new ArrayList<Double>();
	public static ArrayList<Double> twoCubeBR = new ArrayList<Double>();
	public static ArrayList<Double> twoCubeFL = new ArrayList<Double>();
	public static ArrayList<Double> twoCubeBL = new ArrayList<Double>();
	public static ArrayList<Double> twoCubeElv = new ArrayList<Double>();
	public static ArrayList<Double> twoCubeGrab = new ArrayList<Double>();
	public static ArrayList<Double> twoCubeEjectPiston = new ArrayList<Double>();
	public static ArrayList<Double> twoCubeGrabPiston = new ArrayList<Double>();
	public static ArrayList<ArrayList<Double>> twoCubeList = new ArrayList<ArrayList<Double>>();
	
		
	public static void populate(int i) {
		twoCubeFR.add(0.0);
		
		
		
		
		
		twoCubeList.add(twoCubeFR);
		twoCubeList.add(twoCubeBR);
		twoCubeList.add(twoCubeFL);
		twoCubeList.add(twoCubeBL);
		twoCubeList.add(twoCubeElv);
		twoCubeList.add(twoCubeGrab);
		twoCubeList.add(twoCubeEjectPiston);
		twoCubeList.add(twoCubeGrabPiston);
		
		for(ArrayList<Double> bob : twoCubeList ) {
			Initialization.FR.set(bob.get(i));
		}
	}
}
