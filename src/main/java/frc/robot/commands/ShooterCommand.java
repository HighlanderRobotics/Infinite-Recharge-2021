package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;



public class ShooterCommand extends CommandBase {
  public static Shooter shooter;
  public int timesExecuted = 0;
  public long startTime;
  public int threeSecondCount;
  public int increaseRPM;
  public double targetRPM;

  public ShooterCommand(Shooter shooter) {
    ShooterCommand.shooter = shooter;
      addRequirements(shooter);
    }

    @Override
    public void initialize() {
        
        startTime = System.currentTimeMillis();
        Constants.timesExecuted = 0;
       //shooter.hoodMotor.set(0.1);

       //shooter.setRPM(4000);

       //2000 rpm = 
    }

    @Override
    public void execute() { 
     shooter.setAngle(45);
      //System.out.println("Potentiometer Angle:" + shooter.getPotentiometerAngle());
    
     // shooter.firstMotor.set(TalonFXControlMode.PercentOutput, 0.5);
      //shooter.secondMotor.set(TalonFXControlMode.PercentOutput, 0.5);
    // System.out.println(encoderRate);


    
    /*shooter.setRPM(1000 + increaseRPM);
    timesExecuted += 1;
    threeSecondCount = (int)(timesExecuted / 15);
    increaseRPM = threeSecondCount * 100;*/
    

System.out.println(shooter.firstMotor.getSelectedSensorVelocity() * (1.0/2048.0) * 600.0);
    }
  
    @Override
    public void end(boolean interrupted) {
      //System.out.println("Time to reach RPM: " + (System.currentTimeMillis() - startTime));
    shooter.firstMotor.set(TalonFXControlMode.Disabled, 0);
    }
  
    @Override
    public boolean isFinished() {
     
        return false;
    }
  }
