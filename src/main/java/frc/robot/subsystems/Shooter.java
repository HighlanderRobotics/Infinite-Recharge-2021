package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.controller.PIDController;





//manually change the targetVoltage in Constants.java


public class Shooter extends SubsystemBase {
  public final TalonFX firstMotor;
  public final TalonFX secondMotor;
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
  
  
 firstMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocity.kF, Constants.kTimeoutMs);
 firstMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocity.kP, Constants.kTimeoutMs);
 firstMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocity.kI, Constants.kTimeoutMs);
 firstMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocity.kD, Constants.kTimeoutMs);
 
 firstMotor.configNominalOutputForward(0, 20);
 firstMotor.configNominalOutputReverse(0, 20);

 firstMotor.configPeakOutputForward(1, 20);
 firstMotor.configPeakOutputReverse(-1, 20);

 firstMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 10, 0.5));

 secondMotor.follow(firstMotor);
 secondMotor.setInverted(true);
  }


  public boolean isRPMInRange(){
    return (Math.abs(convertVelocitytoRPM(firstMotor.getSelectedSensorVelocity()) - currentSetPoint)) < 50;
  }
  public static double convertVelocitytoRPM(double velocity){
    return velocity * (1.0/2048.0) * 600.0;
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
    //System.out.println("Target RPM:" + targetRPM);
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