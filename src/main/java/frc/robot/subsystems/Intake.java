/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;


public class Intake extends SubsystemBase implements Loggable {
  /**
   * Creates a new IntakeSubsytem.
   */

  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(20, 1, 0);
  ShuffleboardTab tab = Shuffleboard.getTab("Drive Readouts");
  NetworkTableEntry isIntakeOut = 
    tab.add("Intake Status", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withPosition(6, 0)
    .withSize(3,2)
    .withProperties(Map.of("Color when true", "#FFFF00"))
    .getEntry();

    public void extend(){
    intakeSolenoid.set(kReverse);
    }
    public void retract(){
    intakeSolenoid.set(kForward);
    }
  
  public final VictorSPX intakeMotor = new VictorSPX(Constants.INTAKESUBSYSTEM_VICTOR);


  public Intake() {
    addChild("Intake Solenoid", intakeSolenoid);
  }

  @Log
  public double getMotorSpeed(){
    return intakeMotor.getSelectedSensorVelocity();
  }
    

  public void zeroSpeed() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void quarterSpeed() {
    intakeMotor.set(ControlMode.PercentOutput, -0.25);
  }

  public void halfSpeed() {
    intakeMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public void threeQuarterSpeed() {
    intakeMotor.set(ControlMode.PercentOutput, -0.75);
  }

  public void fullSpeed() {
    intakeMotor.set(ControlMode.PercentOutput, -1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    isIntakeOut.setBoolean(intakeSolenoid.get() == kForward);
  }
}
