package frc.robot.subsystems;

import frc.robot.Constants;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



//Circle Thingy = routing carosel purple wheel for moving balls
public class Extractor extends SubsystemBase{
   

    DoubleSolenoid firstExtractorSolenoid = new DoubleSolenoid(20, 2, 3);
    
    public void extend(){
    firstExtractorSolenoid.set(kForward);
    }
    public void retract(){
    firstExtractorSolenoid.set(kReverse);
    }
}
