package frc.robot.subsystems;

import frc.robot.Constants;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



//Circle Thingy = routing carosel purple wheel for moving balls
public class Spindexer extends SubsystemBase{
    @Log
    public DutyCycleEncoder spindexerEncoder = new DutyCycleEncoder(4);
    public VictorSPX circleMotorVictorSPX = new VictorSPX(Constants.circleThingyVictorID);
    private ShuffleboardTab tab = Shuffleboard.getTab("Drive Readouts");
    
    public Spindexer() {
        circleMotorVictorSPX.configPeakOutputForward(0.35);
        circleMotorVictorSPX.configPeakOutputReverse(-0.35);
        addChild("Spindexer Encoder", spindexerEncoder);
        
    }
    /*public void spinUp(double percentage) {
        

    }*/


}
