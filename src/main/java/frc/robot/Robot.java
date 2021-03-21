/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.LinearFilter;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final SwerveDrive m_swerve = new SwerveDrive();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  private final LinearFilter m_xspeedAverage = LinearFilter.movingAverage(10);
  private final LinearFilter m_yspeedAverage = LinearFilter.movingAverage(10);
  private final LinearFilter m_rotAverage = LinearFilter.movingAverage(10);

 @Override
  public void robotPeriodic() {
    Logger.updateEntries();
  }

  @Override
  public void robotInit() {
    Logger.configureLoggingAndConfig(this, false);
  }
  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_controller.getX(GenericHID.Hand.kLeft) * SwerveDrive.kMaxSpeed;
        /*-m_xspeedLimiter.calculate(m_xspeedAverage.calculate(m_controller.getY(GenericHID.Hand.kLeft)))
            * SwerveDrive.kMaxSpeed;*/

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_controller.getY(GenericHID.Hand.kLeft) * SwerveDrive.kMaxSpeed;
        /*-m_yspeedLimiter.calculate(m_yspeedAverage.calculate(m_controller.getX(GenericHID.Hand.kLeft)))
            * SwerveDrive.kMaxSpeed; */
            

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.`
    final var rot = m_controller.getX(GenericHID.Hand.kRight) * SwerveDrive.kMaxAngularSpeed;
        /*-m_rotLimiter.calculate(m_rotAverage.calculate(m_controller.getX(GenericHID.Hand.kRight)))
            * SwerveDrive.kMaxAngularSpeed;*/

    m_swerve.drive(xSpeed, ySpeed, rot, false);
    //m_swerve.drive(0, 0, 0, false);
  }

}
