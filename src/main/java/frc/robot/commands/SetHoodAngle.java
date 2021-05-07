package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.subsystems.HoodAngle;



public class SetHoodAngle extends CommandBase {
  public HoodAngle hoodAngle;
  public LimeLightSubsystem limelight;

  public SetHoodAngle(HoodAngle hoodAngle, LimeLightSubsystem limelight) {
    this.hoodAngle = hoodAngle;
    this.limelight = limelight;
      addRequirements(hoodAngle);
    
    }

    @Override
    public void initialize() {
        
       //shooter.hoodMotor.set(0.1);

    }

    @Override
    public void execute() { 
      //hoodAngle.hoodMotor.set(0.2);
      
     hoodAngle.setAngle(45);
     
      //System.out.println("Potentiometer Angle:" + shooter.getPotentiometerAngle());

    }
  
    @Override
    public void end(boolean interrupted) {
        hoodAngle.hoodMotor.disable();
      //System.out.println("Time to reach RPM: " + (System.currentTimeMillis() - startTime));
    }
  
    @Override
    public boolean isFinished() {
      return hoodAngle.calculateAngleError() < 0.25;
    }
  }
