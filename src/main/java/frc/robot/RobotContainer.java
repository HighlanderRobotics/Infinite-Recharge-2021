/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

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
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.SpinCircleThingy;
import frc.robot.commands.ColorWheelApproach;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.DistanceSensorSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.CircleThingy;
import frc.robot.subsystems.ClimberSubsystem;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
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

    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

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
    private final CircleThingy circleThingy = new CircleThingy();


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        SmartDashboard.putData("AutoAim", new AutoAim(m_swerve, limelight));

        Logger.configureLoggingAndConfig(this, false);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        

        // m_functionsController button uses
        // whileHeldFuncController(Button.kB, m_pneumaticsSubsystem, m_pneumaticsSubsystem::extendControlPanelPiston);
        whileHeldFuncController(Button.kA, m_intakeSubsystem, m_intakeSubsystem::threeQuarterSpeed);

       
       // whileHeldFuncController(Button.kBumperLeft, m_shooterSubsystem, m_shooterSubsystem::shootBalls);
        // whileHeldFuncController(Button.kBumperRight, m_pneumaticsSubsystem, m_pneumaticsSubsystem::extendIntakePiston);

        new JoystickButton(m_functionsController, Button.kA.value)
            .whileHeld(new RaiseHook(3, 45, m_climberSubsystem));
        new JoystickButton(m_functionsController, Button.kX.value)
            .toggleWhenPressed(new ShooterCommand(shooter, limelight));
        new JoystickButton(m_functionsController, Button.kBumperRight.value)
            .whenPressed(() -> shooter.increaseRPM(25));
        new JoystickButton(m_functionsController, Button.kBumperLeft.value)
            .whenPressed(() -> shooter.decreaseRPM(25));
        new JoystickButton(m_functionsController, Button.kB.value)
            .whileHeld(new SpinCircleThingy(circleThingy));

        //new JoystickButton(m_functionsController, Button.kBumperRight.value)
         //   .toggleWhenPressed(new Climb(m_climberSubsystem));
        
        // Driver Controller
        //new JoystickButton(m_driverController, Button.kBumperLeft.value)
        //    .whileHeld(new SensorSlowCommand(m_distanceSensorSubsystem, m_driveSubsystem, teleOpDriveFn));

        //new JoystickButton(m_driverController, Button.kBumperLeft.value)
        //    .whenPressed(() ->     NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0))
        //    .whenReleased(() ->    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1));

        


        // Defaults
        m_swerve.setDefaultCommand(new RunCommand(teleOpDriveFn, m_swerve));
     
        //m_driveSubsystem.setDefaultCommand(new RunCommand(() -> m_driveSubsystem.driveStraight(), m_driveSubsystem));
        limelight.setDefaultCommand(new RunCommand(() -> limelight.lightOn(), limelight));
        // m_pneumaticsSubsystem.setDefaultCommand(new RunCommand(() -> m_pneumaticsSubsystem.retractBothPistons(), m_pneumaticsSubsystem));
  

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
    
        m_swerve.drive(xSpeed, ySpeed, rot, true);
        //m_swerve.drive(0, 0, 0, false);
      }
    

}
