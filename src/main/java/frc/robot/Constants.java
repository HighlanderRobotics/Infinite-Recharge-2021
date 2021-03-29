/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import frc.robot.subsystems.DriveSubsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class HenrysCarpet {

    }

    public final class CompetitionCarpet {

    }

    // Characterization Constants (EDIT LATER)
        // Feedforward/feedback gains
    public static final double ksVolts = 1.04;
    public static final double kvVoltSecondsPerMeter = 3.19;
    public static final double kaVoltSecondsSquaredPerMeter = 0.336; //0.336
    public static double kPDriveVel = 0.00302;
        // Differential Drive Kinematics
    public static final double kTrackWidthMeters = 0.6187672283855629 ; //0.61214 calculated, 0.741 is from characterization
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
        // Max Trajectory Velocity/Acceleration
    public static double kMaxSpeedMetersPerSecond = 8.0;
    public static double kMaxAccelerationMetersPerSecondSquared = 0.5;
        //Ramsete
    public static double kRamseteB = 2;
    public static double kRamseteZeta = 0.7;
    


	// Controllers
    public static int FUNCTIONS_CONTROLLER_PORT = 0;
    public static int DRIVER_CONTROLLER_PORT = 1;

    // Driving multipliers
    public static double HALF_SPEED_MULTIPLIER = 0.5;
    public static double SLOW_TURN_MULTIPLE = 0.75;

    // Distance Sensors
    public static int FRONT_PING_CHANNEL = 4;
    public static int FRONT_ECHO_CHANNEL = 5;
    public static int CONTROLPANEL_PING_CHANNEL = 8;
    public static int CONTROLPANEL_ECHO_CHANNEL = 9;

    // Motors
    public static int DRIVESUBSYSTEM_LEFT_BACK_TALON = 0;
    public static int DRIVESUBSYSTEM_LEFT_FRONT_VICTOR = 1;
    public static int DRIVESUBSYSTEM_RIGHT_BACK_TALON = 3;
    public static int DRIVESUBSYSTEM_RIGHT_FRONT_VICTOR = 2;
    public static int INTAKESUBSYSTEM_VICTOR = 7;
    public static int CLIMBERSUBSYSTEM_WINCH_TALON = 8;
    public static int CLIMBERSUBSYSTEM_WHEEL_TALON = 9;
	public static int circleThingyVictorID = 11;
    public static int talonFirstChannel = 13;
    public static int talonSecondChannel = 14;
    public static int deviceIDCANSparkMax = 15; //hoodMotor


    // Solenoids
    public static final int INTAKE_FORWARD_CHANNEL = 4;
    public static final int INTAKE_REVERSE_CHANNEL = 5;
    public static final int CONTROLPANEL_FORWARD_CHANNEL = 7;
    public static final int CONTROLPANEL_REVERSE_CHANNEL = 6;
    
    // Limiters
    public static final double SLEW_SPEED_LIMITER = 4;
    public static final double SLEW_ROTATION_LIMITER = 3.5;

	public static boolean kGyroReversed = false;
	public static int[] kLeftEncoderPorts = {0,3};
    public static int[] kRightEncoderPorts = {6,7};
    public static double kEncoderPulses = 1000;
    public static double kEncoderDistancePerPulse = 0.47879/kEncoderPulses;
    public static boolean kEncoderReversed = false;

    // Climber Encoders
    public static int[] kWinchEncoderPorts = {5,4};
    public static int[] kWheelEncoderPorts = {1,2};
    public static double kEncoderCyclesPerRevolution = 2048.0;
    public static double kWheelCircumference = 2 * Math.PI * 0.0508;
    public static double kEncoderDistancePerPulseWheel = kWheelCircumference/kEncoderCyclesPerRevolution;
    public static double kHookFullExtension = 10; // Need to calculate
    


    // Analog Channels
	public static int hoodAnglePotentiometerAnalogInputID = 0;





    //Shooter + Routing + Hood Angle
   
	//public static int timesExecuted = 0;

	
	public static final int kSlotIdx = 0;
	public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 20;

	//min possible angle of the hood
	public static int lowerBoundPotentiometer = 32;

	//max possible angle of the hood
	public static int upperBoundPotentiometer = 90;

	//for hoodMotor
    //???? check this
	 
	 	//													kP   kI   kD   kF                  PeakOut */
     public final static Gains kGains_Velocity  = new Gains(3.5, 0.00001, 0.0, (1023 * 0.5) / 9400.0,  1.00);
     
     
     public final static Gains kGains_Hood = new Gains(-0.25, 0.00001, 0, 0, 1.00);
     //fix kF and peakOut
}




/* Shooter Empirical Testing:
hood angle: fully extended

6m: ~4300 (50% accuracy into back port)

5m: ~4675 ????

3m:

2m:



1
-----
angle: 2.5 in between metal and point of purple part (perpendicular)

distance: 280 cm

2750 rpm 

ty = -7.4

-----

2
-----

3000 rpm

ty = -14


3
-----
3300 rpm

ty = -17.7



4
-----
3900 rpm

ty = -21.5


5
-----
rpm

ty = 
*/
