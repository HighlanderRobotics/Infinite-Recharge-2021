/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Represents a swerve drive style drivetrain.
 */
public class SwerveDrive extends SubsystemBase implements Loggable{
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.404, 0.404);
  private final Translation2d m_frontRightLocation = new Translation2d(0.404, -0.404);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.404, 0.404);
  private final Translation2d m_backRightLocation = new Translation2d(-0.404, -0.404);

  private final SwerveModule m_frontLeft = new SwerveModule(2, 1, 22, 282); //2,1
  private final SwerveModule m_frontRight = new SwerveModule(8, 7, 23, 154); //8.7 (subtract 180 to fix direction)
  private final SwerveModule m_backLeft = new SwerveModule(4, 3, 20, 88); //4,3 (was 67)
  private final SwerveModule m_backRight = new SwerveModule(6, 5, 21, 16); //6,5 (was 37)

  @Log public static final ADIS16448_IMU m_gyro = new ADIS16448_IMU();
  public static Pose2d m_pose = new Pose2d(5.0, 13.5, new Rotation2d());
  private final Field2d m_field = new Field2d();

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
  );

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle(), m_pose);

  public SwerveDrive() {
    m_gyro.reset();
    SmartDashboard.putData("Field", m_field);
  }

  /**
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive.
    return Rotation2d.fromDegrees(-m_gyro.getAngle());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates = m_kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot)
    );
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    
  }


  /**
   * Updates the field relative position of the robot.
   */
  public void updateOdometry() {
    m_pose = m_odometry.update(
        getAngle(),
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_backLeft.getState(),
        m_backRight.getState()
    );
  }
  
  @Override
  public void periodic() {
    // automatically update odometry
    updateOdometry();
    m_field.setRobotPose(m_odometry.getPoseMeters());
  }
}