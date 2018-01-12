/***************************************************************************************
*    Title: Gearaffes Robotics Team: FIRST Robotics Competition 2018 Code
*    Authors: Tejas Priyadarshi, Christopher Seiler, Anoop Bhat
*    Contact: http://www.frc5404.org/
***************************************************************************************/
package org.usfirst.frc.team5404.robot;

public class PortIO {
	//----------------------------------------PWM PORTS-------------------------------------------//
		
		// Drive Motor Controllers
	public static final int FR = 0;	
	public static final int BR = 1;
	public static final int  FL = 2;
	public static final int  BL = 3;
	
	// Elevator Motor Controller
	public static final int elv = 4;
		
	//----------------------------------------DIO PORTS-------------------------------------------//

		// Encoders
	
		public static final int rdEncoder1 = 0;
		public static final int rdEncoder2 = 1;

		public static final int ldEncoder1 = 2;
		public static final int ldEncoder2 = 3;
		
		public static final int elevatorEncoder1 = 4;
		public static final int elevatorEncoder2 = 5;
		
		
	//------------------------------------------PCM Ports---------------------------------------------//

		// Solenoids
		
	//------------------------------------------USB Ports---------------------------------------------//

		// Joysticks
		public static final int driver = 0;
		public static final int operator = 1;
		
		
		//Camera
}
