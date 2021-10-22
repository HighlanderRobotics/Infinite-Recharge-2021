package frc.robot.commands;

import frc.robot.subsystems.Spindexer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;


public class SpinSpindexer extends CommandBase{
    public static Spindexer spindexer;

    double lastPosition = 0.0;

    public SpinSpindexer(Spindexer spindexer) {
        SpinSpindexer.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() { 
        //sets speed of motor (that controls the purple routing wheel) to 35%

        spindexer.circleMotorVictorSPX.set(VictorSPXControlMode.PercentOutput, 0.35);
    }

    @Override
    public void execute() { 
        double position = spindexer.spindexerEncoder.get();
        if (Math.abs(position - lastPosition) < 5.0) {
            System.out.println("Jam? spindexer only moved by " + Math.abs(position - lastPosition));
        }
        lastPosition = position;
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
