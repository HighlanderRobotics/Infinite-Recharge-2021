/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.LimeLightSubsystem;
import io.github.oblarg.oblog.annotations.Log;



public class AutoAim extends CommandBase {


  private final SwerveDrive swerveDrive;
  private final LimeLightSubsystem m_limeLightSubsystem;
  private PIDController autoAimPID;

  /**
   * Creates a new autoAim.
   */
  public AutoAim(SwerveDrive swerveDrive, LimeLightSubsystem limelightSubsystem) {
    this.swerveDrive = swerveDrive;
    m_limeLightSubsystem = limelightSubsystem;
    autoAimPID = new PIDController(0.075, 0.06, 0);
    addRequirements(swerveDrive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limeLightSubsystem.lightOn();
    autoAimPID.reset();
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
   
    // auto aiming using PID
    if(!isFinished()) {
      double speed = autoAimPID.calculate(m_limeLightSubsystem.getHorizontalOffset(), 0);
      swerveDrive.drive(0,0, -speed, false);
    }else{
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

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_limeLightSubsystem.getHorizontalOffset()) < 0.1;
  }
}