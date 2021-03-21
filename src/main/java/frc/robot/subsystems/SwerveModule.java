/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotBase;
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
  private static final double kCircumference = kWheelRadius * 2 * Math.PI;
  private static final double kDriveRatio = 8.16;
  private static final double kTurningRatio = 12.8;
  private static final int kEncoderResolution = 2048;
  double targetVelocity = 1 * 2048 / 600; // X RPM 

  private static final double kModuleMaxAngularVelocity = SwerveDrive.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration
      = 2 * Math.PI; // radians per second squared
  
  @Log
  private WPI_TalonFX m_driveMotor;
   private double drivekP = 0.01;
   private double drivekI = 0;
   private double drivekD = 0;
   private double drivekF = 0;
  @Log
  private WPI_TalonFX m_turningMotor;
   private double turningkP = 0.00015;
   private double turningkI = 0;
   private double turningkD = 0;
   private double turningkF = .025;
  private final int driveMotorChannel;
  private final int turningMotorChannel;

@Override
public String configureLogName() {
  return "SwerveModule " + driveMotorChannel + "-" + turningMotorChannel;
}


void setDrivePIDF( double p, 
 double i, 
 double d, 
 double f){
  drivekP = p;
  drivekI = i;
  drivekD = d;
  drivekF = f;
  m_driveMotor.config_kP(0, drivekP);
  m_driveMotor.config_kI(0, drivekI);
  m_driveMotor.config_kD(0, drivekD);
  m_driveMotor.config_kF(0, drivekF);
}


void setTurningPIDF( double p, 
 double i, 
 double d, 
 double f){
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
      m_turningMotor = new WPI_TalonFX(turningMotorChannel);
      m_driveMotor.configFactoryDefault();
      m_turningMotor.configFactoryDefault();
      m_turningMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 20);
      m_turningMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
      //m_turningMotor.setSensorPhase(PhaseSensor);
      //m_turningMotor.setInverted(true);
      
    
      m_turningMotor.setNeutralMode(NeutralMode.Brake);
    setDrivePIDF(0.00015,0,0,0.048);
    setTurningPIDF(1.25,0.0,0,0.048);

    // 50% power to turning - gets 10610 units/100ms
    // 50% power to driving - 10700 units/100ms



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
    , getAngle());
  }

  public Rotation2d getAngle() {
    return new Rotation2d((2*Math.PI/(2048*kTurningRatio))*(m_turningMotor.getSelectedSensorPosition()%(2048*kTurningRatio)));
  }
  @Log.Graph
  public double getAngleDegrees() {
    return 360*(m_turningMotor.getSelectedSensorPosition()%(2048*kTurningRatio))/(2048*kTurningRatio);
  }
  @Log.Graph
  public double getTurningVolts() {
    return m_turningMotor.getMotorOutputVoltage();
  }

  @Log.Graph
  public double inputAngle;

  @Log.Graph
  public double setpoint;

  @Log.Graph
  public double inputVelocity;

    
  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state, getAngle());
    long nearestDegree = Math.round(state.angle.getDegrees());
    
    double setTurnValue = (2048/360.0)*nearestDegree;

    // Calculate the drive output from the drive PID controller.
    //2048 encoder ticks per rotation, input is m/s, we want ticks per 100ms
    inputVelocity = 2048/(10*kCircumference)*state.speedMetersPerSecond*kDriveRatio;
    m_driveMotor.set(TalonFXControlMode.Velocity, inputVelocity);

    // Calculate the turning motor output from the turning PID controller.
    //2048 encoder ticks per rotation, 2pi radians per rotation, so the conversion factor is 2048/2pi radians
    inputAngle = nearestDegree;
    setpoint = setTurnValue*kTurningRatio;
    m_turningMotor.set(TalonFXControlMode.Position, setpoint);
    // System.out.print(inputAngle + "\t");
  }
}
