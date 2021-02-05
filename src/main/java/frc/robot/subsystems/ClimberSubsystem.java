/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimberSubsystem.
   */

  // To-do: initialize relevant motors/climber mechanism parts
  private Encoder encoderString = new Encoder(0,1); //Change 0,1 to relevant ports from Constants
  private Encoder encoderWheel = new Encoder(2, 3); //Change 2,3 to relevant ports from Constants

  //Add two motors

  // Add two duplicate encoders and two duplicate motors

  public ClimberSubsystem() {

    // Need to configure encoders here
    encoderString.reset();
    encoderWheel.reset();
    // encoderString.setDistancePerPulse(distancePerPulse); 
    // Need to calculate this based on the diameter of the shaft - will calculate distance angularly and translate I'm assuming
    // Need clarification on how one would normally calculate this with pulses - confused
    // encoderString.setMinRate(double minRate); will depend on friction - when considered stopped
    // minRate is in distance per second
    // encoderString.setMaxPeriod(double maxPeriod); Set the max period for stopped detection
    // maxPeriod will be the maximum time between rising and falling edges before the FPGA will report the device stopped - expressed in seconds
    // encoderWheel.setDistancePerPulse();
    
  }

  // Raises the climber to grab the hook and hang (to-do)
  public void hang() {


  }

  // Lowers the climber back to the ground (to-do)
  public void lower() {


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
