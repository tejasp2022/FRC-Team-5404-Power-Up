package org.usfirst.frc.team5404.robot;

public class RecordingList {
	
	public static void setRobotValues(int i) {
		if(i<Initialization.FRList.size()) {
			Initialization.FR.set(Initialization.FRList.get(i));
			Initialization.BR.set(Initialization.BRList.get(i));
			Initialization.FL.set(Initialization.FLList.get(i));
			Initialization.BL.set(Initialization.BLList.get(i));
			Initialization.elevator.set(Initialization.elvList.get(i));
			Initialization.grabberMotorController.set(Initialization.grabberList.get(i));
			Initialization.endEffectorPiston.set(Initialization.ejectPistonList.get(i));
			Initialization.grabberPiston.set(Initialization.grabberPistonList.get(i));
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
		Initialization.FRList.clear();
		Initialization.BRList.clear();
		Initialization.FLList.clear();
		Initialization.BLList.clear();
		Initialization.elvList.clear();
		Initialization.grabberList.clear();
		Initialization.ejectPistonList.clear();
		Initialization.grabberPistonList.clear();
		
		Initialization.FRList.add(0.0);
		
		
		
		Initialization.BRList.add(0.0);
		
		
		
		Initialization.FLList.add(0.0);
		
		
		
		Initialization.BLList.add(0.0);
		
		
		
		Initialization.elvList.add(0.0);
		
		
		
		Initialization.grabberList.add(0.0);
		
		
		
		Initialization.ejectPistonList.add(false);
		
		
		
		Initialization.grabberPistonList.add(false);
	}
	
	public static void populateThreeCube() {
		Initialization.FRList.clear();
		Initialization.BRList.clear();
		Initialization.FLList.clear();
		Initialization.BLList.clear();
		Initialization.elvList.clear();
		Initialization.grabberList.clear();
		Initialization.ejectPistonList.clear();
		Initialization.grabberPistonList.clear();
		
		Initialization.FRList.add(0.0);
		
		
		
		Initialization.BRList.add(0.0);
		
		
		
		Initialization.FLList.add(0.0);
		
		
		
		Initialization.BLList.add(0.0);
		
		
		
		Initialization.elvList.add(0.0);
		
		
		
		Initialization.grabberList.add(0.0);
		
		
		
		Initialization.ejectPistonList.add(false);
		
		
		
		Initialization.grabberPistonList.add(false);
	}
	

}