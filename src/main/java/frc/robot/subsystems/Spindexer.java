package frc.robot.subsystems;

import frc.robot.Constants;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DutyCycleEncoder;



//Circle Thingy = routing carosel purple wheel for moving balls
public class Spindexer extends SubsystemBase{
    @Log
    public DutyCycleEncoder spindexerEncoder = new DutyCycleEncoder(4);
    public VictorSPX circleMotorVictorSPX = new VictorSPX(Constants.circleThingyVictorID);
    public Spindexer() {
        circleMotorVictorSPX.configPeakOutputForward(0.35);
        circleMotorVictorSPX.configPeakOutputReverse(-0.35);
        circleMotorVictorSPX.setNeutralMode(NeutralMode.Brake);
    }
    /*public void spinUp(double percentage) {
        

    }*/


}
