/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class SwerveModule implements Loggable{
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 4096;
  double targetVelocity = 1 * 2048 / 600; // X RPM 

  private static final double kModuleMaxAngularVelocity = SwerveDrive.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration
      = 2 * Math.PI; // radians per second squared

  @Log
  private final WPI_TalonFX m_driveMotor;
   private double drivekP = 0.25;
   private double drivekI = 0;
   private double drivekD = 0;
   private double drivekF = 0;
  @Log
  private final WPI_TalonFX m_turningMotor;
   private double turningkP = 0.25;
   private double turningkI = 0;
   private double turningkD = 0;
   private double turningkF = 0;
  private final int driveMotorChannel;
  private final int turningMotorChannel;

@Override
public String configureLogName() {
  return "SwerveModule " + driveMotorChannel + "-" + turningMotorChannel;
}

@Config
void setDrivePIDF(@Config.NumberSlider(name = "p", min = 0, max = 1) double p, 
@Config.NumberSlider(name = "i", min = 0, max = 1) double i, 
@Config.NumberSlider(name = "d", min = 0, max = 1) double d, 
@Config.NumberSlider(name = "f", min = 0, max = 1) double f){
  drivekP = p;
  drivekI = i;
  drivekD = d;
  drivekF = f;
  m_driveMotor.config_kP(0, drivekP);
  m_driveMotor.config_kI(0, drivekI);
  m_driveMotor.config_kD(0, drivekD);
  m_driveMotor.config_kF(0, drivekF);
}

@Config
void setTurningPIDF(@Config.NumberSlider(name = "p", min = 0, max = 1) double p, 
@Config.NumberSlider(name = "i", min = 0, max = 1) double i, 
@Config.NumberSlider(name = "d", min = 0, max = 1) double d, 
@Config.NumberSlider(name = "f", min = 0, max = 1) double f){
  turningkP = p;
  turningkI = i;
  turningkD = d;
  turningkF = f;
  m_turningMotor.config_kP(0, turningkP);
  m_turningMotor.config_kI(0, turningkI);
  m_turningMotor.config_kD(0, turningkD);
  m_turningMotor.config_kF(0, turningkF);
}


  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel) {
    this.driveMotorChannel = driveMotorChannel;
    this.turningMotorChannel = turningMotorChannel;
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    setDrivePIDF(0.25,0,0,0);
    m_turningMotor = new WPI_TalonFX(turningMotorChannel);
    setTurningPIDF(0.25, 0, 0, 0);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
   // m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    //m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity()
    , new Rotation2d(2*Math.PI*m_turningMotor.getSelectedSensorPosition()/2048));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    
    m_driveMotor.set(TalonFXControlMode.Velocity, state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    //2048 encoder ticks per rotation, 2pi radians per rotation, so the conversion factor is 2048/2pi radians
    m_turningMotor.set(TalonFXControlMode.Position, 2048/(2*Math.PI*state.angle.getRadians()));
  }
}
