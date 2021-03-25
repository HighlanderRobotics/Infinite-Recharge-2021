/**
 *  Class that organizes gains used when assigning values to slots
 */
package frc.robot;

public class Gains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final double kPeakOutput;
	
	public Gains(double _kP, double _kI, double _kD, double _kF, double _kPeakOutput){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kPeakOutput = _kPeakOutput;
	}
}