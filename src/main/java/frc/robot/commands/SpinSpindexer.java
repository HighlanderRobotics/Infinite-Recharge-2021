package frc.robot.commands;

import frc.robot.subsystems.Spindexer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;


public class SpinSpindexer extends CommandBase{
    public static Spindexer spindexer;

    public SpinSpindexer(Spindexer spindexer) {
        SpinSpindexer.spindexer = spindexer;
        addRequirements(spindexer);
    }

@Override
public void initialize() { 
    //sets speed of motor (that controls the purple routing wheel) to 20%

spindexer.circleMotorVictorSPX.set(VictorSPXControlMode.PercentOutput, -0.2);
}

@Override
    public void execute() { 

}

@Override
        public void end(boolean interrupted) {
            //disables motor once B button (or whatever button this command has been bound to) is no longer being held or is toggled off somehow
            spindexer.circleMotorVictorSPX.set(VictorSPXControlMode.Disabled, 0);
}

@Override
    public boolean isFinished() {
        return false;
    }
}
