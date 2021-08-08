/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Intake extends SubsystemBase {
  /**
   * Creates a new IntakeSubsytem.
   */

  DoubleSolenoid intakeSolenoid = new DoubleSolenoid(20, 0, 1);
    

    public void extend(){
    intakeSolenoid.set(kForward);
    }
    public void retract(){
    intakeSolenoid.set(kReverse);
    }
  
  private final VictorSPX intakeMotor = new VictorSPX(Constants.INTAKESUBSYSTEM_VICTOR);


  public Intake() {}
    

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
  }
}
