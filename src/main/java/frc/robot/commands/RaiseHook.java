// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RaiseHook extends CommandBase {
  /** Creates a new RaiseHook. */
  private final ClimberSubsystem m_climberSubsystem;

  public RaiseHook(ClimberSubsystem climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climberSubsystem = climberSubsystem;
    addRequirements(m_climberSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double wheelSpeed =  (0.1 / 25.0) * m_climberSubsystem.getDistanceWheelEncoder() + 0.4;
    m_climberSubsystem.setWheelSpeed(wheelSpeed);
    if (m_climberSubsystem.getAngleWinchEncoder() > getDesiredAngle()) {
      m_climberSubsystem.setWinchSpeed(wheelSpeed * 3);

    }
      /** 
    if (m_climberSubsystem.getDistanceWheelEncoder() < 55) {
      m_climberSubsystem.setWinchSpeed(wheelSpeed);
    }
    else {
      m_climberSubsystem.setWinchSpeed(wheelSpeed * 1.5);
    }
    */
    else { m_climberSubsystem.setWinchSpeed(wheelSpeed); 
    }
  }

  public double getDesiredAngle() {


    return 0.000881 * Math.pow(m_climberSubsystem.getDistanceWheelEncoder(), 3) 
      - 0.121 * Math.pow(m_climberSubsystem.getDistanceWheelEncoder(), 2) + 21.5;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubsystem.setWinchSpeed(0);
    // Unwinch to have hook grab on to the bar - not for now
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climberSubsystem.getDistanceWheelEncoder() >= 65.0; // full hook extension
  }
}
