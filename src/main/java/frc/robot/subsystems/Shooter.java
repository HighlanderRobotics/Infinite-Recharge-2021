/* 
The shooter subsystem has two main goals. The first is to declare, build, and configure all of the shooter
paraphernalia (motors, encoders/sensors). The second purpose is to provide commands to regulate RPM of 
the 2 shooter motor. 




*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class Shooter extends SubsystemBase {
  public final TalonFX firstMotor;
  public final TalonFX secondMotor;
  public PIDController hoodPIDController;
  

  public Shooter(){ 
    //declare motors
    firstMotor = new TalonFX(Constants.talonFirstChannel);
    secondMotor = new TalonFX(Constants.talonSecondChannel);
  
    


    /*configuring TalonFX motors, nothing here should need changing (ever), the only things to change would
    be the PID constant values, but these can be changed in the Constants.java
    */

    firstMotor.configFactoryDefault();
    firstMotor.configNeutralDeadband(0.001);
    firstMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
      
    //assigns PID config values (using constant values in Constants.java)
    firstMotor.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocity.kF, Constants.kTimeoutMs);
    firstMotor.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocity.kP, Constants.kTimeoutMs);
    firstMotor.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocity.kI, Constants.kTimeoutMs);
    firstMotor.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocity.kD, Constants.kTimeoutMs);
    
    firstMotor.configNominalOutputForward(0, 20);
    firstMotor.configNominalOutputReverse(0, 20);

    firstMotor.configPeakOutputForward(1, 20);
    firstMotor.configPeakOutputReverse(-1, 20);

    firstMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 10, 0.5));

    firstMotor.setNeutralMode(NeutralMode.Coast);
    secondMotor.setNeutralMode(NeutralMode.Coast);

    //configures the 2 shooter motors work together
    secondMotor.follow(firstMotor);
    secondMotor.setInverted(true);
  }



//isRPMInRange is a simple command that can be called (continuously) to determine when the RPM gets into target range
  public boolean isRPMInRange(){
    return (Math.abs(convertVelocitytoRPM(firstMotor.getSelectedSensorVelocity()) - currentSetPoint)) < 200;
  }

/*convertVelocityToRPM simply converts the encoder units to RPM
double velocity is in units/100ms and the integrated encoder is based on 2048 units/revolution, so to convert 
from targetRPM to targetVelocity:
(targetRPM) / ((100ms per 1 second = 10) (sec per min = 60)) 
   */
  public static double convertVelocitytoRPM(double velocity){
    return velocity * (1.0/2048.0) * 600.0;
  }
 

  double currentSensorVelocity;
  double currentSetPoint;

  
  //targetVelocity is in pulses/100 ms (as opposed to 2048 pulses/revoluion)

  /*setRPM is a simple command which is able to take the target RPM (of the shooter motors) as an argument, convert
  from RPM to velocity and assign the desired velocity to the motors (secondMotor will follow the firstMotor as
  previously configured)
  */
  public void setRPM (double targetRPM){
    double targetVelocity = (targetRPM * 2048) / 600;
    currentSetPoint = targetRPM;
    System.out.println("Target RPM:" + targetRPM);
    firstMotor.set(TalonFXControlMode.Velocity, targetVelocity);

  }

  /*increase or decrease RPM by an externally defined increment, used to manually increase/decrease RPM while 
  robot is running, these commands are intended to be used only for diagnostics and testing by assigning each
  to a button, so that re-building is not necessary to change the RPM
  */
  public void increaseRPM (int increment){
    setRPM(currentSetPoint + increment);
  }
  public void decreaseRPM (int decrement){
    setRPM(currentSetPoint - decrement);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter rpm", convertVelocitytoRPM(firstMotor.getSelectedSensorVelocity()));
  }
}