package org.usfirst.frc.team5404.robot;

public class Prefs {
	public static double getDouble(String key, double backup) {
		if(Initialization.prefs.containsKey(key)) {
			return Initialization.prefs.getDouble(key, backup);
		} else {
			Initialization.prefs.putDouble(key, backup);
			return backup;
		}
	}
	public static boolean getBoolean(String key, boolean backup) {
		if(Initialization.prefs.containsKey(key)) {
			return Initialization.prefs.getBoolean(key, backup);
		} else {
			Initialization.prefs.putBoolean(key, backup);
			return backup;
		}
	}
	public static int getInt(String key, int backup) {
		if(Initialization.prefs.containsKey(key)) {
			return Initialization.prefs.getInt(key, backup);
		} else {
			Initialization.prefs.putInt(key, backup);
			return backup;
		}
	}
	public static String getString(String key, String backup) {
		if(Initialization.prefs.containsKey(key)) {
			return Initialization.prefs.getString(key, backup);
		} else {
			Initialization.prefs.putString(key, backup);
			return backup;
		}
	}
}