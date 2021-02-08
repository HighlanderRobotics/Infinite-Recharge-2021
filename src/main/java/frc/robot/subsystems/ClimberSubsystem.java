/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.controller.PIDController;


public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimberSubsystem.
   */

  // To-do: initialize relevant motors/climber mechanism parts
  private Encoder encoderString = new Encoder(Constants.kStringEncoderPorts[0], Constants.kStringEncoderPorts[1]);
  private Encoder encoderWheel = new Encoder(Constants.kWheelEncoderPorts[0], Constants.kWheelEncoderPorts[1]);

  //Add two motors
  private VictorSPX stringMotor = new VictorSPX(Constants.CLIMBERSUBSYSTEM_STRING_VICTOR);
  private VictorSPX wheelMotor = new VictorSPX(Constants.CLIMBERSUBSYSTEM_WHEEL_VICTOR);

  // Add two duplicate encoders and two duplicate motors
  // TBD because of motor shortage.

  private PIDController winchPID = new PIDController(0,0,0);
  // Need to input PID constants from 

  double ratio = 0.5; // Multiply string speed by this to have wheel speed.

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

  public void resetEncoders() {
    
    encoderString.reset();
    encoderWheel.reset();
  }

  public void setSpeed(double speed) {

    stringMotor.set(ControlMode.PercentOutput, speed * ratio);
    wheelMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setRatio(double ratio) {

    this.ratio = ratio;
  }

  public void slowWinch() {

    ratio -= .05;
  }

  public void fastWinch() {

    ratio += .05;
  }

  /**
  public void fullPowerString() {

    stringMotor.set(ControlMode.PercentOutput, 1);
  }

  public void fullPowerWheel() {

    wheelMotor.set(ControlMode.PercentOutput, 1);
  }
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
