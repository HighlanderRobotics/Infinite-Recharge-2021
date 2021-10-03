// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Spindexer;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpinSpindexerToPosition extends PIDCommand {
  /** Creates a new SpinSpindexerToPosition. */
  public SpinSpindexerToPosition(Spindexer spindexer, double targetPosition) {

    super(
        // The controller that the command will use
        new PIDController(2.0, 0, 0),
        // This should return the measurement
        () -> spindexer.spindexerEncoder.get(),
        // This should return the setpoint (can also be a constant)
        targetPosition,
        // This uses the output
        output -> spindexer.circleMotorVictorSPX.set(VictorSPXControlMode.PercentOutput, -output)
          // Use the output here
        );
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(spindexer);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return getController().atSetpoint();
  }
}
