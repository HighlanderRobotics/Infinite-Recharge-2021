package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Extractor;
import frc.robot.subsystems.HoodAngle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.SwerveDrive;

public class ShootWithoutLimelight extends ParallelDeadlineGroup {
    public ShootWithoutLimelight(final SwerveDrive swerve, final double angle, final Shooter shooter, 
                            final Spindexer spindexer, final Extractor extractor, final Intake intake, final HoodAngle hoodAngle) {
        super(new SequentialCommandGroup(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                        new InstantCommand(() -> hoodAngle.setTargetAngle(angle)),
                        new SequentialCommandGroup(
                            new WaitUntilCommand(shooter::isRPMInRange)
                        ),
                        new SpinSpindexerToPosition(spindexer, Constants.spindexerStart)
                    ),
                    new ParallelCommandGroup(
                        new RunCommand(extractor::extend, extractor),
                        new SpinSpindexer(spindexer))),
                new InstantCommand(() -> shooter.setRPM(4000), shooter))
        ),
        new RunCommand(intake::extend, intake));
    }
}
