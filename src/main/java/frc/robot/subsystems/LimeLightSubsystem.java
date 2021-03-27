/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Shooter;


public class LimeLightSubsystem extends SubsystemBase {
  public Shooter shooter;
  public double horizontalOffset; 
  public double verticalOffset;
  public double areaOffset;
  NetworkTable table;
  public double x;
  public double y;
  public boolean isPointingAtTarget;
  
  public LimeLightSubsystem() {
    lightOn();
  }

 
  public void defaultReadings() {
    

    //read values periodically
    x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    double area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);


    
  }

  public void lightReadings() {
    
  }

  public double getArea() {
    return areaOffset;
  }

  public double getHorizontalOffset() {
    return horizontalOffset;
  }

  public double getVerticalOffset() {
    return verticalOffset;
  }

  public void lightOn() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
  }

  public void lightOff() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
  }

  public boolean isPointingAtTarget() {
    return isPointingAtTarget;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double x = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    double y = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    double area = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
    isPointingAtTarget = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0) == 1;
    SmartDashboard.putNumber("limelightX", x);
    SmartDashboard.putNumber("limelightY", y);
    SmartDashboard.putNumber("limelightArea", area);
    SmartDashboard.putBoolean("is limelight detecting target", isPointingAtTarget);

    horizontalOffset = x;
    verticalOffset = y;

  }

}
