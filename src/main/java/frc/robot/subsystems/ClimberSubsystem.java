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
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimberSubsystem.
   */

  // To-do: initialize relevant motors/climber mechanism parts
  private Encoder winchEncoder = new Encoder(Constants.kWinchEncoderPorts[0], Constants.kWinchEncoderPorts[1]);
  private Encoder wheelEncoder = new Encoder(Constants.kWheelEncoderPorts[0], Constants.kWheelEncoderPorts[1]);
  private Encoder winchMotorEncoder = new Encoder(Constants.kWinchMotorEncoderPorts[0], Constants.kWinchMotorEncoderPorts[1]);

  private Servo ratchetServo = new Servo(Constants.kWinchServoPort);

  //Add two motors
  private WPI_TalonSRX winchMotor = new WPI_TalonSRX(Constants.CLIMBERSUBSYSTEM_WINCH_TALON);
  private WPI_TalonSRX wheelMotor = new WPI_TalonSRX(Constants.CLIMBERSUBSYSTEM_WHEEL_TALON);

  // Ratchet Solenoid
  // private Solenoid ratchet = new Solenoid(Constants.CLIMBER_RATCHET_CHANNEL);

  // Add two duplicate encoders and two duplicate motors
  // TBD because of motor shortage.

  //private PIDController winchPID = new PIDController(0,0,0);
  // private PIDController wheelPID = new PIDController(0,0,0);
  // Need to input PID constants from 

  double wheelSpeed = 0;
  double winchSpeed = 0;

  boolean ratchetPower = false;

  public ClimberSubsystem() {

    // Need to configure encoders here
    winchEncoder.reset();
    wheelEncoder.reset();
    winchMotorEncoder.reset();
    winchMotorEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulseWheel);
    winchEncoder.setDistancePerPulse(360.0 / Constants.kEncoderCyclesPerRevolution); 
    // encoderWinch.setMinRate(double minRate); will depend on friction - when considered stopped
    // minRate is in distance per second
    // encoderWinch.setMaxPeriod(double maxPeriod); Set the max period for stopped detection
    // maxPeriod will be the maximum time between rising and falling edges before the FPGA will report the device stopped - expressed in seconds
    wheelEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulseWheel);
    wheelEncoder.setMaxPeriod(.1); //encoder configures itself stopped after .1 seconds.
    // ratchet.set(ratchetPower);
    
  }

  public void turnOnWinch() {

    winchMotor.set(ControlMode.PercentOutput, 0.3);
  }

  public void setRatchetServo(double angle) {

    ratchetServo.setPosition(angle); // Goes from -1.0 to 1.0 range
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

    if (wheelSpeed - speed >= -1.0) {
      
      wheelSpeed -= speed;
    }
    else {

      wheelSpeed = -1.0;
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

    if (winchSpeed - speed >= -1.0) {
      
      winchSpeed -= speed;
    }
    else {

      winchSpeed = -1.0;
    }
    winchMotor.set(ControlMode.PercentOutput, winchSpeed);
  }

  public void resetEncoders() {
    
    winchEncoder.reset();
    wheelEncoder.reset();
  }

  public double getDistanceWheelEncoder() {

    return wheelEncoder.getDistance();
  }

  public double getAngleWinchEncoder() {

    return winchEncoder.getDistance();
  }

  public double getDistanceWinchEncoder() {

    return winchMotorEncoder.getDistance();
    // gives negative values to extend
  }
/** 
  public void ratchetPowerSwitch() {

    if (ratchetPower) { ratchetPower = false; }
    else if (!ratchetPower) { ratchetPower = true; }
    ratchet.set(ratchetPower);
  }

  public void setWinchPIDSetpoint(double setpoint) {

    winchPID.setSetpoint(setpoint);
  }

  public void resetWinchPID() {

    winchPID.reset();
  }
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Climber Height", this.getDistanceWheelEncoder());
    SmartDashboard.putNumber("Winch Angle", this.getAngleWinchEncoder());
    SmartDashboard.putNumber("Winch Distance", this.getDistanceWinchEncoder());
    SmartDashboard.putNumber("Wheel Motor Power", this.wheelSpeed);
    SmartDashboard.putNumber("Winch Motor Power", this.winchSpeed);
  }
}
