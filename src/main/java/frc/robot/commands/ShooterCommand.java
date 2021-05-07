package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.subsystems.HoodAngle;



public class ShooterCommand extends CommandBase {
  public Shooter shooter;
  public HoodAngle hoodAngle;
  public LimeLightSubsystem limelight;
  public int timesExecuted = 0;
  public long startTime;
  public int threeSecondCount;
  public int increaseRPM;
  public double targetRPM;
  

  public ShooterCommand(Shooter shooter, HoodAngle hoodAngle, LimeLightSubsystem limelight) {
    this.shooter = shooter;
    this.limelight = limelight;
    this.hoodAngle = hoodAngle;
      addRequirements(shooter);
    
    }

    @Override
    public void initialize() {
        
        startTime = System.currentTimeMillis();
       //shooter.hoodMotor.set(0.1);

      

      shooter.setRPM(4000);

       //2000 rpm = 
    }

    @Override
    public void execute() { 

    //change the argument on following line to desired hood angle in degrees
    hoodAngle.setAngle(45);
    





/*    Autoaiming by changing hood angle:

      double x = limelight.getVerticalOffset();
      //double targetRPM = (5.3155 * Math.pow(x, 2)) + (76.5261 * x) + 3010.1;
      double targetAngle = (-0.0514 * Math.pow(x, 2)) + (-1.586 * x) + 56.858;

      hoodAngle.setAngle(targetAngle); 



      -------------------- */

    //1st zone: all 3 into hexagon, none into backport
    //2nd zone: first two into hexagon, sometimes into backport, but third always hits bottom rim
    //3rd zone: all 3 into hex with ~2 into backport
    //4th zone: all 3 into hex with ~2 into backport


      //shooter.setRPM(targetRPM);
      
      
     //shooter.setAngle(45);
      //System.out.println("Potentiometer Angle:" + shooter.getPotentiometerAngle());
    
     // shooter.firstMotor.set(TalonFXControlMode.PercentOutput, 0.5);
      //shooter.secondMotor.set(TalonFXControlMode.PercentOutput, 0.5);
    // System.out.println(encoderRate);


    
    /*shooter.setRPM(1000 + increaseRPM);
    timesExecuted += 1;
    threeSecondCount = (int)(timesExecuted / 15);
    increaseRPM = threeSecondCount * 100;*/
    

System.out.println(Shooter.convertVelocitytoRPM(shooter.firstMotor.getSelectedSensorVelocity()));
    }
  
    @Override
    public void end(boolean interrupted) {
      //System.out.println("Time to reach RPM: " + (System.currentTimeMillis() - startTime));
    }
  
    @Override
    public boolean isFinished() {
     
        return false;
    }
  }
