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
import frc.robot.commands.ControlPanelPosition;
import frc.robot.commands.ControlPanelRotation;
import frc.robot.commands.ColorWheelApproach;
import frc.robot.commands.Climb;
import frc.robot.subsystems.ControlPanelSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.DistanceSensorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;
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
    private final ControlPanelSubsystem m_controlPanelSubsystem = new ControlPanelSubsystem();
    private final LimeLightSubsystem m_limelightSubsystem = new LimeLightSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final PneumaticsSubsystem m_pneumaticsSubsystem = new PneumaticsSubsystem();
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

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

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
        whileHeldFuncController(Button.kB, m_pneumaticsSubsystem, m_pneumaticsSubsystem::extendControlPanelPiston);
        whileHeldFuncController(Button.kA, m_intakeSubsystem, m_intakeSubsystem::threeQuarterSpeed);
        whileHeldFuncController(Button.kBumperLeft, m_shooterSubsystem, m_shooterSubsystem::shootBalls);
        whileHeldFuncController(Button.kBumperRight, m_pneumaticsSubsystem, m_pneumaticsSubsystem::extendIntakePiston);


        new JoystickButton(m_functionsController, Button.kBumperRight.value)
            .toggleWhenPressed(new Climb(m_climberSubsystem));
        
        // Driver Controller
        //new JoystickButton(m_driverController, Button.kBumperLeft.value)
        //    .whileHeld(new SensorSlowCommand(m_distanceSensorSubsystem, m_driveSubsystem, teleOpDriveFn));


        //new JoystickButton(m_driverController, Button.kBumperLeft.value)
        //    .whenPressed(() ->     NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0))
        //    .whenReleased(() ->    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1));

        


        // Defaults
        m_controlPanelSubsystem.setDefaultCommand(new RunCommand(() -> m_controlPanelSubsystem.zeroSpeed(), m_controlPanelSubsystem));
        m_shooterSubsystem.setDefaultCommand(new RunCommand(() -> m_shooterSubsystem.zeroSpeed(), m_shooterSubsystem));
        m_intakeSubsystem.setDefaultCommand(new RunCommand(() -> m_intakeSubsystem.zeroSpeed(), m_intakeSubsystem));
        m_swerve.setDefaultCommand(new RunCommand(teleOpDriveFn, m_swerve));
        m_limelightSubsystem.setDefaultCommand(new RunCommand(() -> m_limelightSubsystem.lightOn(), m_limelightSubsystem));
        m_pneumaticsSubsystem.setDefaultCommand(new RunCommand(() -> m_pneumaticsSubsystem.retractBothPistons(), m_pneumaticsSubsystem));
        SmartDashboard.putData("Blue", new InstantCommand(() -> m_controlPanelSubsystem.colorRotation("B")));
        SmartDashboard.putData("Red", new InstantCommand(() -> m_controlPanelSubsystem.colorRotation("R")));
        SmartDashboard.putData("Green", new InstantCommand(() -> m_controlPanelSubsystem.colorRotation("G")));
        SmartDashboard.putData("Yellow", new InstantCommand(() -> m_controlPanelSubsystem.colorRotation("Y")));

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
