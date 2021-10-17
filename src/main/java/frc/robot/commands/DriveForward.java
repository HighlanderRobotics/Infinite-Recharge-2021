// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class DriveForward extends CommandBase {
  private final SwerveDrive drive;

  public DriveForward(SwerveDrive drive) {
    this.drive = drive;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.drive(0.5, 0.0, 0.0, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      drive.drive(0.5, 0.0, 0.0, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      drive.drive(0.0, 0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
