// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class Raise extends CommandBase {
  /** Creates a new Raise. */

  private final ClimberSubsystem m_climberSubsystem;

  public Raise(ClimberSubsystem climberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climberSubsystem = climberSubsystem;
    addRequirements(m_climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // m_climberSubsystem.resetWinchPID();
    // m_climberSubsystem.setSpeed(0.5);
    // m_climberSubsystem.setRatio(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // how to give an option to change speed with buttons
    // do I need separate commands?


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
