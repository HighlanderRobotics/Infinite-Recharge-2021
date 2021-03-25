package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;



//Circle Thingy = routing carosel purple wheel for moving balls
public class CircleThingy extends SubsystemBase{
    public VictorSPX circleMotorVictorSPX = new VictorSPX(Constants.circleThingyVictorID);

    /*public void spinUp(double percentage) {
        

    }*/
}
