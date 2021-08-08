/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/* Temporary scattered notes (probably not useful) about climber to help my successor (-Robbie): (FEEL FREE TO DELETE)
Cable guard, elastic band, automatically wants to roll up

Controls: 1 pusher motor (on one side with two green wheels, one is powered, compliant wheels), 1 motor controls string (to keep from falling back)

Angle arms controls how much you want to pull the string (far back it tilts back)

Goal of the testing: find the right angle for the measurement arm
Based on the angle pull out the string or pull out the cable guard more


Kevin:
Could directly predict winch tension based on how high up the hook had been pushed
By the end, we had a ratio of where the tension line should be pulled compared to how high it was
(simple stepwise function?)
PrepareCommand: when everything starts, hook is slack, needs to be pulled into basic position, so this command pulls the line until there is tension on it, basically an angle measure (an encoder), might find that a simple ratio is equally effective

Preparation works
Extension (generally) works

How to make driver actively put the hook on the bar?

How to allow hook to go up to adjust?


Doing set heights in testing

Bunch of problems to solve, extension working, matter of how do we put it to a specific height? Something with PID? Tension related to height? Separate mode: we’ve extended, now use a different set of joystick bindings? Or something totally different?

Line got caught in the gears cutting the paracord, which is the goal of the guard over the gears


Front plate with a window where the paracord can actually come in, which would come back a bit to make sure it can’t go in the gears, trying to keep it simple. (simple is reasonable)


First, we should figure out exactly how we are going to control the coming down/hanging portion of it? Make user interface better?

More testing?

Climbing only happens at the end, could swap out other controls near the end

Tension is a major concern, need enough tension to prevent the chord from collapsing/falling, but not too much


Not apparent in video in google drive folder: pieces of wood between the links to give extra rigidity

*/
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimberSubsystem.
   */

  // To-do: initialize relevant motors/climber mechanism parts
  private Encoder winchEncoder = new Encoder(Constants.kWinchEncoderPorts[0], Constants.kWinchEncoderPorts[1]);
  private Encoder wheelEncoder = new Encoder(Constants.kWheelEncoderPorts[0], Constants.kWheelEncoderPorts[1]);
  private Encoder winchMotorEncoder = new Encoder(Constants.kWinchMotorEncoderPorts[0], Constants.kWinchMotorEncoderPorts[1]);

  //Add two motors
  private WPI_TalonSRX winchMotor = new WPI_TalonSRX(Constants.CLIMBERSUBSYSTEM_WINCH_TALON);
  private WPI_TalonSRX wheelMotor = new WPI_TalonSRX(Constants.CLIMBERSUBSYSTEM_WHEEL_TALON);

  private Servo ratchetServo = new Servo(Constants.kWinchServoPort);

  // Ratchet Solenoid
  // private Solenoid ratchet = new Solenoid(Constants.CLIMBER_RATCHET_CHANNEL);

  // Add two duplicate encoders and two duplicate motors
  // TBD because of motor shortage.

  double wheelSpeed = 0;
  double winchSpeed = 0;

  boolean ratchetPower = false;

  public ClimberSubsystem() {

    // Need to configure encoders here
    winchEncoder.reset();
    wheelEncoder.reset();
    winchMotorEncoder.reset();
    winchEncoder.setDistancePerPulse(360.0 / Constants.kEncoderCyclesPerRevolution); 
    // encoderWinch.setMinRate(double minRate); will depend on friction - when considered stopped
    // minRate is in distance per second
    // encoderWinch.setMaxPeriod(double maxPeriod); Set the max period for stopped detection
    // maxPeriod will be the maximum time between rising and falling edges before the FPGA will report the device stopped - expressed in seconds
    wheelEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulseWheel);
    wheelEncoder.setMaxPeriod(.1); //encoder configures itself stopped after .1 seconds.
    // ratchet.set(ratchetPower);
    
  }

  public void resetEncoders() {
    
    winchEncoder.reset();
    wheelEncoder.reset();
  }

  public void setRatchetServo(boolean engaged) {

    if (engaged) { ratchetServo.setPosition(0); } // Goes from -1.0 to 1.0 range
    else { ratchetServo.setPosition(0.4); }
  }


  public void setWheelSpeed(double speed) {

    this.wheelSpeed = speed;

    wheelMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setWinchSpeed(double speed) {

    this.winchSpeed = speed;
    winchMotor.set(ControlMode.PercentOutput, speed);
  }

  public double getDistanceWheelEncoder() {

    return wheelEncoder.getDistance() + 4;
  }

  public double getAngleWinchEncoder() {

    return winchEncoder.getDistance();
  }
  public boolean isHookFullyExtended() {

    return (Math.abs(Constants.kHookFullExtension - wheelEncoder.getDistance()) < 0.1);
  }

  public double getDistanceWinchEncoder() {

    return winchMotorEncoder.getDistance();
    // gives negative values to extend
  }

  public void brake() {

    if (getDistanceWheelEncoder() * 0.00833 < 0) {
      setWheelSpeed(0);
    }
    else {
      setWheelSpeed(getDistanceWheelEncoder() * 0.0045);
    }
  }

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
