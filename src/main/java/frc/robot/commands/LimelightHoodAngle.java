/* ShooterCommand.java originally controlled much of the shooter, but as development progressed, this command became
more and more simple. In its current state (ignoring archived code that has been commented out), the ShooterCommand
accomplishes 3 things. 

Firstly, in the initialize, it sets the RPM to a constant 4000 RPM. We eventually opted
for a constant RPM of 4000 with only the hood angle as the variable (so that the parabola representing
the ball's trajectory is as flat as possible at the top, thus using high speeds with lower hood angles).
Adjusting both RPM and hoodAngle could improve accuracy and range of possible shots, however this adds significant
(mathematical) complexity and requires much more testing, so given our limited time frame in Spring 2021, we
use a single variable.

The second purpose is found in the execute portion of the command, which hosts 3 important lines of code that, in
their current state, MUST BE IN THE EXECUTE because they are reliant on being re-called every 20 ms. Together, the
three lines input the current limelight reading, represented by the variable x, and use an equation to find a hood
angle that will allow for an accurate shot into the target, and finally set the hood angle to the calculated value.
It is important to note that the x variable does not represent anything horizontal, but actually represents the 
vertical offset of the reflective tape (below the target) according to the limelight. 

The third and final purpose of this command is to provide diagnostic information for testing, specifically the 
current RPM of the shooter motors. This is accomplished with a simple print line statement at the end of the execute,
thus displaying the current RPM every 20 ms.
*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants;
import frc.robot.subsystems.HoodAngle;



public class LimelightHoodAngle extends CommandBase {
  public Shooter shooter;
  public HoodAngle hoodAngle;
  public LimeLightSubsystem limelight;
  public int timesExecuted = 0;
  public long startTime;
  public int threeSecondCount;
  public int increaseRPM;
  public double targetRPM;
  private double hoodAngleOffset = -5;
  

  public LimelightHoodAngle(HoodAngle hoodAngle, LimeLightSubsystem limelight) {
    this.limelight = limelight;
    this.hoodAngle = hoodAngle;
      addRequirements(hoodAngle);
    
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() { 

      double x = limelight.getVerticalOffset();
      //double targetRPM = (5.3155 * Math.pow(x, 2)) + (76.5261 * x) + 3010.1; (less accurate, archived equation)

      //equation derived from test points which calculates the needed angle for each shot based on limelight readings
      double targetAngle = (-0.0514 * Math.pow(x, 2)) + (-1.586 * x) + 56.858 + hoodAngleOffset;

      hoodAngle.setAngle(targetAngle);

      System.out.println(Shooter.convertVelocitytoRPM(shooter.firstMotor.getSelectedSensorVelocity()));
    }
  
    @Override
    public void end(boolean interrupted) {
      //System.out.println("Time to reach RPM: " + (System.currentTimeMillis() - startTime));
    }
  
    @Override
    public boolean isFinished() {
        return hoodAngle.calculateAngleError() < 0.25;
    }
  }
