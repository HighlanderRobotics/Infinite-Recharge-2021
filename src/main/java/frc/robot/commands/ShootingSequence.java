package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Extractor;
import frc.robot.subsystems.HoodAngle;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.SwerveDrive;

public class ShootingSequence extends SequentialCommandGroup {
    public ShootingSequence(final SwerveDrive swerve, final LimeLightSubsystem limelight, final Shooter shooter, 
                            final Spindexer spindexer, final Extractor extractor, final Intake intake, final HoodAngle hoodAngle) {
        super(
            new SearchingLimelight(swerve, limelight), 
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new WaitUntilCommand(shooter::isRPMInRange),
                        new ParallelCommandGroup(
                            new LimelightHoodAngle(limelight, hoodAngle),
                            new AutoAim(swerve, limelight),
                            new SpinSpindexerToPosition(spindexer, Constants.spindexerStart)
                        )
                    ),
                    new ParallelCommandGroup(
                        new RunCommand(extractor::extend, extractor),
                        new SpinSpindexer(spindexer))),
                new InstantCommand(() -> shooter.setRPM(4000), shooter))
        );
    }
}
