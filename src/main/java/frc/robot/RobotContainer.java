/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

//import org.graalvm.compiler.code.DataSection.ZeroData;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.AutoAim;
import frc.robot.commands.RaiseHook;
import frc.robot.commands.SearchingLimelight;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SpinSpindexer;
import frc.robot.commands.SpinSpindexerToPosition;
import frc.robot.commands.PrepareHook;
import frc.robot.commands.SetHoodAngle;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Extractor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HoodAngle;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.GenericHID.Hand;


/**
 * 
 * 
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private final LimeLightSubsystem limelight = new LimeLightSubsystem();
    // private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();
    // private final DistanceSensorSubsystem m_distanceSensorSubsystem = new DistanceSensorSubsystem();

    private final Intake intake = new Intake();

    // private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
    //commented to disable while climber extension encoders are unplugged, uncomment to fix later

    private final XboxController m_functionsController = new XboxController(Constants.FUNCTIONS_CONTROLLER_PORT);
    private final XboxController m_driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);

    private final Runnable teleOpDriveFn = () -> driveWithJoystick(false);

    private final SwerveDrive m_swerve = new SwerveDrive();


    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

   //from shooter repository
    private final Shooter shooter = new Shooter();
    public static boolean isButtonToggled = false;
    private final Spindexer spindexer = new Spindexer();
    private final HoodAngle hoodAngle = new HoodAngle();
    private final Extractor extractor = new Extractor();


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        SmartDashboard.putData("AutoAim", new AutoAim(m_swerve, limelight));
        SmartDashboard.putData("Roibbie thing", new SequentialCommandGroup(new SearchingLimelight(m_swerve, limelight), new AutoAim(m_swerve, limelight)));
        SmartDashboard.putNumber("Potentiometer Reading in Degrees", hoodAngle.getPotentiometerAngle());

        Logger.configureLoggingAndConfig(this, false);

        
        //SmartDashboard.putData("Search Auto Aim", new SequentialCommandGroup(new SearchingLimelight(m_swerve, limelight), new AutoAim(m_swerve, limelight)));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        
        whileHeldFuncController(Button.kA, intake, intake::threeQuarterSpeed);


        /*autoshooting command group: with a press of the Y button, this command will allow the robot to shoot quite accurately into the high port,
        regardless of the robot's current positon or orientation

        The robot first checks if the limelight can see the target (defined by reflective tape). If it cannot, the robot will spin until it can.
        Once the target can be seen, the robot will align itself to the target so that it has the best chance of making a shot. The robot then waits
        until the RPM is in the desired range (currently set to 4000 RPM). While the robot is aligning itself and reaching the correct RPM, the necessary
        hood angle is determined based on limelight readings, and then set. Now that the robot is properly aligned with the high port and the shooter motors
        are spinning at the correct RPM, the extractor will extend, meanwhile the "circleThingy", aka the purple routing wheel which holds balls, will
        begin to spin. Press Y again once all the balls have been fired, and the command group is over.
        */

        Command shootCommand = 
                new ParallelDeadlineGroup(
                    new SequentialCommandGroup(
                        new SearchingLimelight(m_swerve, limelight), 
                        new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                    new SequentialCommandGroup(
                                        new AutoAim(m_swerve, limelight),
                                        new WaitUntilCommand(shooter::isRPMInRange)
                                    ),
                                    new SpinSpindexerToPosition(spindexer, Constants.spindexerStart)
                                ),
                                new ParallelCommandGroup(
                                    new RunCommand(extractor::extend, extractor),
                                    new SpinSpindexer(spindexer))),
                            new ShooterCommand(shooter, hoodAngle, limelight))
                    ),
                    new RunCommand(intake::extend, intake));

        new JoystickButton(m_driverController, Axis.kRightTrigger.value)
            .toggleWhenPressed(shootCommand);

        new JoystickButton(m_functionsController, Axis.kRightTrigger.value)
            .toggleWhenPressed(shootCommand);

        new JoystickButton(m_functionsController, Button.kBumperRight.value)
            .whileHeld(new RunCommand(() -> spindexer.circleMotorVictorSPX.set(VictorSPXControlMode.PercentOutput, 0.3), spindexer));

        new JoystickButton(m_functionsController, Button.kBumperLeft.value)
            .whileHeld(new RunCommand(() -> spindexer.circleMotorVictorSPX.set(VictorSPXControlMode.PercentOutput, -0.3), spindexer));

        new JoystickButton(m_functionsController, Axis.kLeftTrigger.value)
            .whileHeld(new RunCommand(intake::halfSpeed, intake));

        new JoystickButton(m_functionsController, Button.kA.value)
            .toggleWhenPressed(new RunCommand(intake::extend, intake));


        
        
        // Defaults
        m_swerve.setDefaultCommand(new RunCommand(teleOpDriveFn, m_swerve));
     
        //m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.driveStraight(), m_driveSubsystem));
        limelight.setDefaultCommand(new RunCommand(limelight::lightOn, limelight));

        extractor.setDefaultCommand(new RunCommand(extractor::retract, extractor));
        //defaults extractor to remain up

        intake.setDefaultCommand(new RunCommand(() -> {intake.retract(); intake.zeroSpeed();}, intake));
        //defaults intake to remain up

        //CHANGE THIS FOR SHOOTER RPM
        shooter.setDefaultCommand(new RunCommand(() -> {shooter.firstMotor.set(ControlMode.PercentOutput, 0);}, shooter));

    //   m_climberSubsystem.setDefaultCommand(new RunCommand(() -> { 
    //         m_climberSubsystem.brake();
    //         //m_climberSubsystem.setWheelSpeed(0);
    //         m_climberSubsystem.setWinchSpeed(0);
    //     }, m_climberSubsystem));

        spindexer.setDefaultCommand(new RunCommand(() -> spindexer.circleMotorVictorSPX.set(VictorSPXControlMode.PercentOutput, 0), spindexer));


    }

    public void configureTestBindings() {
        new JoystickButton(m_driverController, Button.kB.value)
        .toggleWhenPressed(new SpinSpindexerToPosition(spindexer, 0.106));
       
        new JoystickButton(m_functionsController, Button.kB.value)
        .toggleWhenPressed(new SetHoodAngle(hoodAngle, limelight));

        new JoystickButton(m_functionsController, Button.kX.value)
            .toggleWhenPressed(new ShooterCommand(shooter, hoodAngle, limelight));
    }

    private void whileHeldFuncController(Button button, Subsystem subsystem, Runnable runnable) {
        new JoystickButton(m_functionsController, button.value).whileHeld(new InstantCommand(runnable, subsystem));


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     * 
     *         public Command getAutonomousCommand() { // An ExampleCommand will run
     *         in autonomous
     * 
     *         }
     */


    
     
    private void driveWithJoystick(boolean fieldRelative) {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        final var xSpeed = m_xspeedLimiter.calculate(m_driverController.getY(GenericHID.Hand.kLeft))
                * SwerveDrive.kMaxSpeed;
    
        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        final var ySpeed = m_yspeedLimiter.calculate(m_driverController.getX(GenericHID.Hand.kLeft))
                * SwerveDrive.kMaxSpeed;
                    
        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.`
        final var rot = m_rotLimiter.calculate(m_driverController.getX(GenericHID.Hand.kRight))
                * SwerveDrive.kMaxAngularSpeed;
    
        m_swerve.drive(xSpeed, ySpeed, rot, false);
        //m_swerve.drive(0, 0, 0, false);
      }
    

}
