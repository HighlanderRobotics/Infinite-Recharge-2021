/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Solenoid;


public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimberSubsystem.
   */

  // To-do: initialize relevant motors/climber mechanism parts
  private Encoder encoderWinch = new Encoder(Constants.kWinchEncoderPorts[0], Constants.kWinchEncoderPorts[1]);
  private Encoder encoderWheel = new Encoder(Constants.kWheelEncoderPorts[0], Constants.kWheelEncoderPorts[1]);

  //Add two motors
  private WPI_TalonSRX winchMotor = new WPI_TalonSRX(Constants.CLIMBERSUBSYSTEM_WINCH_TALON);
  private WPI_TalonSRX wheelMotor = new WPI_TalonSRX(Constants.CLIMBERSUBSYSTEM_WHEEL_TALON);

  // Ratchet Solenoid
  // private Solenoid ratchet = new Solenoid(Constants.CLIMBER_RATCHET_CHANNEL);

  // Add two duplicate encoders and two duplicate motors
  // TBD because of motor shortage.

  private PIDController winchPID = new PIDController(0,0,0);
  // private PIDController wheelPID = new PIDController(0,0,0);
  // Need to input PID constants from 

  double wheelSpeed = 0;
  double winchSpeed = 0;

  boolean ratchetPower = false;

  public ClimberSubsystem() {

    // Need to configure encoders here
    encoderWinch.reset();
    encoderWheel.reset();
    // encoderWinch.setDistancePerPulse(360 / Constants.kEncoderCyclesPerRevolution); 
    // encoderString.setMinRate(double minRate); will depend on friction - when considered stopped
    // minRate is in distance per second
    // encoderString.setMaxPeriod(double maxPeriod); Set the max period for stopped detection
    // maxPeriod will be the maximum time between rising and falling edges before the FPGA will report the device stopped - expressed in seconds
    encoderWheel.setDistancePerPulse(Constants.kEncoderDistancePerPulseWheel);
    encoderWheel.setMaxPeriod(.1); //encoder configures itself stopped after .1 seconds.
    // ratchet.set(ratchetPower);
    
  }

  public void turnOnWinch() {

    winchMotor.set(ControlMode.PercentOutput, 0.3);
  }

  public void stopWheelAndWinch() {

    stopWheel();
    stopWinch();
  }

  public void stopWheel() {

    wheelSpeed = 0;
    wheelMotor.set(ControlMode.PercentOutput, wheelSpeed);
  }

  public void stopWinch() {

    winchSpeed = 0;
    winchMotor.set(ControlMode.PercentOutput, winchSpeed);
  }

  public void increaseWheelSpeed(double speed) {

    if (wheelSpeed + speed <= 1) { 
      
      wheelSpeed += speed; 
    }
    else {

      wheelSpeed = 1;
    }
    wheelMotor.set(ControlMode.PercentOutput, wheelSpeed);
  }

  public void decreaseWheelSpeed(double speed) {

    if (wheelSpeed - speed >= 0) {
      
      wheelSpeed -= speed;
    }
    else {

      wheelSpeed = 0;
    }
    wheelMotor.set(ControlMode.PercentOutput, wheelSpeed);
  }

  public void increaseWinchSpeed(double speed) {

    if (winchSpeed + speed <= 1) { 
      
      winchSpeed += speed; 
    }
    else {

      winchSpeed = 1;
    }
    winchMotor.set(ControlMode.PercentOutput, winchSpeed);
  }

  public void decreaseWinchSpeed(double speed) {

    if (winchSpeed - speed >= 0) {
      
      winchSpeed -= speed;
    }
    else {

      winchSpeed = 0;
    }
    winchMotor.set(ControlMode.PercentOutput, winchSpeed);
  }

  public void resetEncoders() {
    
    encoderWinch.reset();
    encoderWheel.reset();
  }

  public double getDistanceWheelEncoder() {

    return encoderWheel.getDistance();
  }

  public double getDistanceWinchEncoder() {

    return encoderWinch.getDistance();
  }
/** 
  public void ratchetPowerSwitch() {

    if (ratchetPower) { ratchetPower = false; }
    else if (!ratchetPower) { ratchetPower = true; }
    ratchet.set(ratchetPower);
  }
  */

  public void setWinchPIDSetpoint(double setpoint) {

    winchPID.setSetpoint(setpoint);
  }

  public void resetWinchPID() {

    winchPID.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
