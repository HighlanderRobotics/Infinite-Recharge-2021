/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DistanceSensorSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.LimeLightSubsystem;
import io.github.oblarg.oblog.annotations.Log;



public class AutoAim extends CommandBase {


  private static final double maxAngle = 1;
  private final SwerveDrive swerveDrive;
  private final LimeLightSubsystem m_limeLightSubsystem;
  private PIDController turnLeftPID;
  private PIDController turnRightPID;
  private double leftTurnSpeed;
  private double rightTurnSpeed; 
  @Log boolean isAutoAimFinished;
  /**
   * Creates a new autoAim.
   */
  public AutoAim(SwerveDrive swerveDrive, LimeLightSubsystem limelightSubsystem) {
    this.swerveDrive = swerveDrive;
    m_limeLightSubsystem = limelightSubsystem;
    turnLeftPID = new PIDController(1, 0, 0);
    turnRightPID = new PIDController(1, 0, 0);
    addRequirements(swerveDrive , m_limeLightSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isAutoAimFinished = false;
    m_limeLightSubsystem.lightOn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Create PIDController as an instance variable
    // Set kI and kD to 0 intiially
    // Set kP to some small value to start
    //
    //  -30 off
    //  * 0.01 = -0.3
    //  30
    //  * 0.01 = 0.3
    //  
    //  err_value * kP + integral(err_value) * kI + derivative(err_value) * kD
    
    // then adjust for crosshair placement not exactly centered by adding some constant
    leftTurnSpeed = turnLeftPID.calculate(m_limeLightSubsystem.getHorizontalOffset(), 0.25);
    rightTurnSpeed = turnRightPID.calculate(m_limeLightSubsystem.getHorizontalOffset(), -0.25);

    // auto aiming using PID
    if(m_limeLightSubsystem.getHorizontalOffset() > 0) {
      swerveDrive.drive(0,0,leftTurnSpeed, false);
    } else if(m_limeLightSubsystem.getHorizontalOffset() < 0) {
      swerveDrive.drive(0,0, rightTurnSpeed, false);
    }
    else if(m_limeLightSubsystem.isPointingAtTarget()){
      swerveDrive.drive(0, 0, 0, false);
    }

    // no PID auto aim
    /*if(m_limeLightSubsystem.getHorizontalOffset() > maxAngle + 1) {
      swerveDrive.drive(0, 0, 0.4, false);
    } else if(m_limeLightSubsystem.getHorizontalOffset() < -maxAngle + 1) {
      swerveDrive.drive(0, 0, -0.4, false);
    }
    else if(m_limeLightSubsystem.isPointingAtTarget()){
      swerveDrive.drive(0, 0, 0, false);
    }*/
    
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isAutoAimFinished = !interrupted;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_limeLightSubsystem.getHorizontalOffset() < maxAngle && 
    m_limeLightSubsystem.getHorizontalOffset() > -maxAngle;
  }
}