/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5404.robot;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Solenoid class for running high voltage Digital Output on the PCM.
 *
 * <p>The Solenoid class is typically used for pneumatic solenoids, but could be used for any
 * device within the current spec of the PCM.
 */
public class GSolenoid implements Sendable {
 // private final int m_channel; // The channel to control.
  //private int m_solenoidHandle;

  /**
   * Constructor using the default PCM ID (defaults to 0).
   *
   * @param channel The channel on the PCM to control (0..7).
   */
  public GSolenoid(final int channel) {
    //this(SensorBase.getDefaultSolenoidModule(), channel);
  }

  /**
   * Constructor.
   *
   * @param moduleNumber The CAN ID of the PCM the solenoid is attached to.
   * @param channel      The channel on the PCM to control (0..7).
   */
  public GSolenoid(final int moduleNumber, final int channel) {
   /* super(moduleNumber);
    m_channel = channel;

    SensorBase.checkSolenoidModule(m_moduleNumber);
    SensorBase.checkSolenoidChannel(m_channel);

    int portHandle = SolenoidJNI.getPortWithModule((byte) m_moduleNumber, (byte) m_channel);
    m_solenoidHandle = SolenoidJNI.initializeSolenoidPort(portHandle);

    HAL.report(tResourceType.kResourceType_Solenoid, m_channel, m_moduleNumber);
    setName("Solenoid", m_moduleNumber, m_channel);*/
  }

  /**
   * Destructor.
   */
  public synchronized void free() {
   /* super.free();
    SolenoidJNI.freeSolenoidPort(m_solenoidHandle);
    m_solenoidHandle = 0;*/
  }

  /**
   * Set the value of a solenoid.
   *
   * @param on True will turn the solenoid output on. False will turn the solenoid output off.
   */
  private boolean on = false;
  public void set(boolean on) {
	  this.on = on;
    //SolenoidJNI.setSolenoid(m_solenoidHandle, on);
  }

  /**
   * Read the current value of the solenoid.
   *
   * @return True if the solenoid output is on or false if the solenoid output is off.
   */
  public boolean get() {
	  return on;
    //return SolenoidJNI.getSolenoid(m_solenoidHandle);
  }

  /**
   * Check if solenoid is blacklisted. If a solenoid is shorted, it is added to the blacklist and
   * disabled until power cycle, or until faults are cleared.
   *
   * @return If solenoid is disabled due to short.
   * @see #clearAllPCMStickyFaults()
   */
  public boolean isBlackListed() {
	  return false;
  //  int value = getPCMSolenoidBlackList() & (1 << m_channel);
   // return value != 0;
  }

  /**
   * Set the pulse duration in the PCM. This is used in conjunction with
   * the startPulse method to allow the PCM to control the timing of a pulse.
   * The timing can be controlled in 0.01 second increments.
   *
   * @param durationSeconds The duration of the pulse, from 0.01 to 2.55 seconds.
   *
   * @see #startPulse()
   */
  public void setPulseDuration(double durationSeconds) {
   // long durationMS = (long) (durationSeconds * 1000);
   // SolenoidJNI.setOneShotDuration(m_solenoidHandle, durationMS);
  }

  /**
   * Trigger the PCM to generate a pulse of the duration set in
   * setPulseDuration.
   *
   * @see #setPulseDuration(double)
   */
  public void startPulse() {
   // SolenoidJNI.fireOneShot(m_solenoidHandle);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
  //  builder.setSmartDashboardType("Solenoid");
   // builder.setSafeState(() -> set(false));
   // builder.addBooleanProperty("Value", this::get, this::set);
  }

@Override
public String getName() {
	// TODO Auto-generated method stub
	return null;
}

@Override
public void setName(String name) {
	// TODO Auto-generated method stub
	
}

@Override
public String getSubsystem() {
	// TODO Auto-generated method stub
	return null;
}

@Override
public void setSubsystem(String subsystem) {
	// TODO Auto-generated method stub
	
}
}
