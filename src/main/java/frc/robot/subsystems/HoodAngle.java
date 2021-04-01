package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogPotentiometer;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.controller.PIDController;





//manually change the targetVoltage in Constants.java


public class HoodAngle extends SubsystemBase {
 
    public final CANSparkMax hoodMotor;
    public PIDController hoodPIDController;
  

    public HoodAngle(){ 

  // canSparkMAX for controlling hood angle
  
  hoodMotor = new CANSparkMax(Constants.deviceIDCANSparkMax, CANSparkMaxLowLevel.MotorType.kBrushless);

  //intializing + configuring hoodPIDController
    hoodPIDController = new PIDController(Constants.kGains_Hood.kP, Constants.kGains_Hood.kI, Constants.kGains_Hood.kD);
  }


  public AnalogPotentiometer potentiometer = new AnalogPotentiometer(Constants.hoodAnglePotentiometerAnalogInputID, Constants.upperBoundPotentiometer - Constants.lowerBoundPotentiometer, Constants.lowerBoundPotentiometer);

    public double getPotentiometerAngle(){
      return potentiometer.get(); //* (Constants.upperBoundPotentiometer - Constants.lowerBoundPotentiometer) + Constants.lowerBoundPotentiometer;
    }
  
  //setAngle should be called periodically in order for PID control to occur
  public void setAngle(double targetAngle){
    hoodPIDController.setSetpoint(targetAngle);
    double hoodMotorSpeed = hoodPIDController.calculate(getPotentiometerAngle());
  

    //hoodMotor should not exceed 10% output, so this prevents it from exceeding 8% (to be safe)
    if(hoodMotorSpeed > 0.08) {
      hoodMotorSpeed = 0.08;
    }else if(hoodMotorSpeed < -0.08){
      hoodMotorSpeed = -0.08;
    }
    hoodMotor.set(hoodMotorSpeed);
    
  }

  public double calculateAngleError(){
    return(Math.abs(hoodPIDController.getSetpoint() - getPotentiometerAngle()));
  
  }
  public void increaseHoodAngle (double increment){
    setAngle(getPotentiometerAngle() + increment);
    System.out.println(getPotentiometerAngle());
  }
  public void decreaseHoodAngle (double decrement){
    setAngle(getPotentiometerAngle() - decrement);
    System.out.println(getPotentiometerAngle());
  }
}