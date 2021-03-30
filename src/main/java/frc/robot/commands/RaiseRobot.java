// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class RaiseRobot extends CommandBase {
  /** Creates a new RaiseRobot. */
  private final ClimberSubsystem m_climberSubsystem;
  private boolean finished = false;

  public RaiseRobot(ClimberSubsystem climberSubsystem) {
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

    // m_climberSubsystem.setWheelSpeed(-0.2); // Sets climber to retract
    if (m_climberSubsystem.getDistanceWheelEncoder() < 5) { // if climber is extended < 5 meters

      // m_climberSubsystem.setWheelSpeed(0); // stop retracting
      finished = true;
    }
    //Using wheelEncoder and wheel PID, climb to a predetermined setpoint based on Climber team input
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // boolean method in climber subsystem for if climber is at appropriate height
    return finished;
  }
}
