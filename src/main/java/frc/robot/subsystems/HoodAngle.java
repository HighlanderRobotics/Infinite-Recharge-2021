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
import edu.wpi.first.wpilibj.interfaces.Potentiometer;





//manually change the targetVoltage in Constants.java


public class HoodAngle extends SubsystemBase {
 
  public static double potentiometerError = 13.33;
	//min possible angle of the hood
	public static double lowerBoundPotentiometer = 32 - potentiometerError;

	//max possible angle of the hood
  public static double upperBoundPotentiometer = 90 - potentiometerError;
    
  public final CANSparkMax hoodMotor;
  public PIDController hoodPIDController;
  public double targetAngle = 0;

  

  public HoodAngle(){

  // canSparkMAX for controlling hood angle
  
  hoodMotor = new CANSparkMax(Constants.deviceIDCANSparkMax, CANSparkMaxLowLevel.MotorType.kBrushless);

  //intializing + configuring hoodPIDController
    hoodPIDController = new PIDController(Constants.kGains_Hood.kP, Constants.kGains_Hood.kI, Constants.kGains_Hood.kD);

    addChild("Hood Angle PID", hoodPIDController);
    addChild("Potentiometer", potentiometer);
    //addChild("Hood Angle Motor", hoodMotor);
  }


  public AnalogPotentiometer potentiometer = new AnalogPotentiometer(Constants.hoodAnglePotentiometerAnalogInputID, upperBoundPotentiometer - lowerBoundPotentiometer, lowerBoundPotentiometer);

    public double getPotentiometerAngle(){
      return potentiometer.get(); //* (Constants.upperBoundPotentiometer - Constants.lowerBoundPotentiometer) + Constants.lowerBoundPotentiometer;
    }
  
  //setAngle should be called periodically in order for PID control to occur
  public void setAngle(double targetAngle){
    // make sure angle is within physical limits
    targetAngle = Math.max(targetAngle, Constants.hoodMin);
    targetAngle = Math.min(targetAngle, Constants.hoodMax);

    hoodPIDController.setSetpoint(targetAngle);
    double hoodMotorSpeed = hoodPIDController.calculate(getPotentiometerAngle());
    
    //hoodMotor should not exceed 10% output, so this prevents it from exceeding 8% (to be safe)
    double maxHood = 0.25;
    if(hoodMotorSpeed > maxHood) {
      hoodMotorSpeed = maxHood;
    }else if(hoodMotorSpeed < -maxHood){
      hoodMotorSpeed = -maxHood;
    }
    hoodMotor.set(hoodMotorSpeed);
    System.out.println(getPotentiometerAngle());
    
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
  public void setTargetAngle (double targetAngle){
      this.targetAngle = targetAngle;
  }
}