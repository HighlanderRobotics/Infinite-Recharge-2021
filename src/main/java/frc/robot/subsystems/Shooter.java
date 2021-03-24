package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogPotentiometer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.controller.PIDController;





//manually change the targetVoltage in Constants.java


public class Shooter extends SubsystemBase {
  public final TalonFX firstMotor;
  public final TalonFX secondMotor;


 
 public final CANSparkMax hoodMotor;


public PIDController hoodPIDController;
  

  public Shooter(){ 

  firstMotor = new TalonFX(Constants.talonFirstChannel);
  secondMotor = new TalonFX(Constants.talonSecondChannel);
  

  //configuring TalonFX motors 
   /* Factory Default all hardware to prevent unexpected behaviour */
   firstMotor.configFactoryDefault();
		
   /* Config neutral deadband to be the smallest possible */
   firstMotor.configNeutralDeadband(0.001);
   firstMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  
  
 firstMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
 firstMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
 firstMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
 firstMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
 
 firstMotor.configNominalOutputForward(0, 20);
 firstMotor.configNominalOutputReverse(0, 20);

 firstMotor.configPeakOutputForward(1, 20);
 firstMotor.configPeakOutputReverse(-1, 20);

 firstMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 10, 0.5));

 secondMotor.follow(firstMotor);
 secondMotor.setInverted(true);

  
  

  // canSparkMAX for controlling hood angle
  
  hoodMotor = new CANSparkMax(Constants.deviceIDCANSparkMax, CANSparkMaxLowLevel.MotorType.kBrushless);

  //intializing + configuring hoodPIDController
    hoodPIDController = new PIDController(Constants.kP, Constants.kI, Constants.kD);
  }


  public AnalogPotentiometer potentiometer = new AnalogPotentiometer(Constants.hoodAnglePotentiometerAnalogInputID, Constants.upperBoundPotentiometer - Constants.lowerBoundPotentiometer, 0);

    public double getPotentiometerAngle(){
      return potentiometer.get(); //* (Constants.upperBoundPotentiometer - Constants.lowerBoundPotentiometer) + Constants.lowerBoundPotentiometer;
    }
  
  //setAngle should be called periodically in order for PID control to occur
  public void setAngle(double targetAngle){
    hoodPIDController.setSetpoint(targetAngle);
    double hoodMotorSpeed = hoodPIDController.calculate(getPotentiometerAngle());

    //hoodMotor should not exceed 10% output, so this prevents it from exceeding 8% (to be safe)
    if (hoodMotorSpeed > 0.08 || hoodMotorSpeed < -0.08) {
      if(hoodMotorSpeed > 0) {
        hoodMotorSpeed = 0.08;
      }else{
        hoodMotorSpeed = -0.08;
      }
    }
    hoodMotor.set(hoodMotorSpeed);
    if(calculateAngleError() < 1.5){
      hoodMotor.disable();
    }
  }

  public double calculateAngleError(){
    return(Math.abs(hoodPIDController.getSetpoint() - getPotentiometerAngle()));
  }

  
  /*targetVelocity is in units/100ms and the integrated encoder is based on 2048 units/revolution, so to convert from targetRPM to targetVelocity, 
   *(targetRPM) / ((100ms per 1 second = 10) (sec per min = 60)) 
   */

   //factor in gear ratio? (with wheels) ~*~

  double currentSensorVelocity;
  double currentSetPoint;

  //targetVelocity is in pulses/100 ms (as opposed to 2048 pulses/revoluion)a
   
     // cert statement could function as a failsafe if necessary

  
  public void setRPM (double targetRPM){
    double targetVelocity = (targetRPM * 2048) / 600;
    currentSetPoint = targetRPM;
    System.out.println("Target Velocity:" + targetVelocity);
    firstMotor.set(TalonFXControlMode.Velocity, targetVelocity);
  }

    	/* Configured for Velocity Closed Loop on Integrated Sensors' Sum and Arbitrary FeedForward on joyX */
			
			/* Uncomment to view RPM in Driver Station */
            // double actual_RPM = (_rightMaster.getSelectedSensorVelocity() / (double)Constants.kSensorUnitsPerRotation * 600f);
            // System.out.println("Vel[RPM]: " + actual_RPM + " Pos: " + _rightMaster.getSelectedSensorPosition());
  
  public void increaseRPM (int increment){
    setRPM(currentSetPoint + increment);
  }
  public void decreaseRPM (int decrement){
    setRPM(currentSetPoint - decrement);
  }
}