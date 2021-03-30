package frc.robot.commands;

import frc.robot.subsystems.CircleThingy;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;


public class SpinCircleThingy extends CommandBase{
    public static CircleThingy circleThingy;

    public SpinCircleThingy(CircleThingy circleThingy) {
        SpinCircleThingy.circleThingy = circleThingy;
        addRequirements(circleThingy);
    }

@Override
public void initialize() { 
    //sets speed of motor (that controls the purple routing wheel) to 20%

circleThingy.circleMotorVictorSPX.set(VictorSPXControlMode.PercentOutput, 0.6);
}

@Override
    public void execute() { 

}

@Override
        public void end(boolean interrupted) {
            //disables motor once B button (or whatever button this command has been bound to) is no longer being held or is toggled off somehow
            circleThingy.circleMotorVictorSPX.set(VictorSPXControlMode.Disabled, 0);
}

@Override
    public boolean isFinished() {
        return false;
    }
}
