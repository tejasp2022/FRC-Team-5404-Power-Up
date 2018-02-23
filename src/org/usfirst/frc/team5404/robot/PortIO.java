/***************************************************************************************
*    Title: Gearaffes Robotics Team FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

public class PortIO {
	//----------------------------------------PWM PORTS-------------------------------------------//
		// Drive Motor Controllers
			public static final int FR = 0;	
			public static final int BR = 1;
			public static final int FL = 2;
			public static final int BL = 3;
	
		// Elevator Motor Controller
			public static final int elevator = 4;
			
		// Intake Motor Controller
			public static final int grabberMotor = 5;
			
		// Intake Motor Controller
			//public static final int intakeMotorRight = 6;
			//public static final int intakeMotorLeft = 7;
		
		// Climber Motor Controller 
			public static final int climberMotor = 6;
		
	//----------------------------------------DIO PORTS-------------------------------------------//
		// Encoders
			public static final int rdEncoder1 = 0;
			public static final int rdEncoder2 = 1;

			public static final int ldEncoder1 = 2;
			public static final int ldEncoder2 = 3;
		
			public static final int elevatorEncoder1 = 4;
			public static final int elevatorEncoder2 = 5;
			
			public static final int bottomLimitSwitch = 6;
			public static final int topLimitSwitch = 7;
			
			public static final int grabberEncoder1 = 8;
			public static final int grabberEncoder2 = 9;
			
	//------------------------------------------PCM Ports---------------------------------------------//
		// Range Finder
			public static final int rangeFinder = 0;
		
	//------------------------------------------PCM Ports---------------------------------------------//
		// Solenoids
	        public static final int endEffector = 0;
	        public static final int grabber = 1;
	        public static final int brake1 = 2;
	        public static final int brake2 = 3;
	        public static final int climber1 = 6;
	        public static final int climber2 = 7;
	        public static final int detacher1 = 4;
	        public static final int detacher2 = 5;
	        
		
	//------------------------------------------USB Ports---------------------------------------------//
		// Joysticks
			public static final int driver = 0;
			public static final int operator = 1;
}