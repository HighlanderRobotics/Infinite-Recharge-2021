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

    m_climberSubsystem.resetEncoders();
    m_climberSubsystem.resetWinchPID();
    // Should set the setpoint here
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // set motor speeds based on winch PIDController
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Unwinch to have hook grab on to the bar
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climberSubsystem.isHookFullyExtended();
  }
}
