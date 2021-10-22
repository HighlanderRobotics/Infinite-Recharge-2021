package frc.robot.subsystems;

import frc.robot.Constants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



//Circle Thingy = routing carosel purple wheel for moving balls
public class Extractor extends SubsystemBase implements Loggable{
   

    DoubleSolenoid firstExtractorSolenoid = new DoubleSolenoid(20, 2, 3);
    
    public void extend(){
    firstExtractorSolenoid.set(kReverse);
    }
    public void retract(){
    firstExtractorSolenoid.set(kForward);
    }

    @Log
    public boolean isExtended(){
        return firstExtractorSolenoid.get() == kReverse;
    }
}
