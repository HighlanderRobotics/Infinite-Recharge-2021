// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RaiseHook extends CommandBase {
  /** Creates a new RaiseHook. */
  private final ClimberSubsystem m_climberSubsystem;
  private double distance;
  private double angle;

  public RaiseHook(double distance, double angle, ClimberSubsystem climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climberSubsystem = climberSubsystem;
    addRequirements(m_climberSubsystem);
    this.distance = distance;
    this.angle = angle;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //m_climberSubsystem.resetEncoders();
    //m_climberSubsystem.resetWinchPID();
    m_climberSubsystem.setWheelSpeed(.1);
    // Should set the setpoint here
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if ((m_climberSubsystem.getDistanceWheelEncoder() - distance) > .2) { // climber too tall
      m_climberSubsystem.setWheelSpeed(-0.1); // Set the wheel motor to retract climber at speed of .1
    }
    else if ((m_climberSubsystem.getDistanceWheelEncoder() - distance) < -.2) { // climber too short
      m_climberSubsystem.setWheelSpeed(0.1); // Set the wheel motor to extend climber at speed of .1
    }
    else { // climber just right :D
      m_climberSubsystem.setWheelSpeed(0);
      if (m_climberSubsystem.getAngleWinchEncoder() - angle < 10) { // Angle too small - too little tension
        m_climberSubsystem.setWinchSpeed(-0.1); // Sets winch motor to retract
      }
      else if (m_climberSubsystem.getAngleWinchEncoder() - angle > 10) { // Angle too large - too much tension
        m_climberSubsystem.setWinchSpeed(0.1); // Sets winch motor to extend
      }
    }
    /**
     * if (m_climberSubsystem.getDistanceWheelEncoder() <= TableValue1) {
     * 
     *  m_climberSubsystem.setWinchSetpoint(double setpoint - angle);
     * }
     * else if (m_climberSubsystem.getDistanceWheelEncoder() > TableValue1 && m_climberSubsystem.getDistanceWheelEncoder() <= Table Value2) {
     *  m_climberSubsystem.setWinchSetpoint(double setpoint - angle);
     * }
     * else if (m_climberSubsystem.getDistanceWheelEncoder() >= end value)
     * 
     * Repeat for all of table values
     * 
     * 
     */
    // set motor speeds based on winch PIDController
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_climberSubsystem.setWheelSpeed(0);
    m_climberSubsystem.setWinchSpeed(0);
    // Unwinch to have hook grab on to the bar - not for now
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_climberSubsystem.getAngleWinchEncoder() - angle) <= 10;
  }
}
