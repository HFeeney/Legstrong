// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.Swerve.*;

public class SwerveModule extends SubsystemBase {
  
  private int moduleNumber;
  private double setSpeed;

  private CANSparkMax speedMotor;
  private WPI_TalonSRX angleMotor;
  private AnalogInput angleEncoder;

  private ProfiledPIDController anglePID;
  private PIDController speedPID;
  private SimpleMotorFeedforward angleFeedforward;

  /** Creates a new SwerveModule. */
  public SwerveModule(int moduleNumber) {

    this.moduleNumber = moduleNumber;
    this.setSpeed = 0.0;
    
    speedMotor = new CANSparkMax(SPEED_MOTOR_PORTS[moduleNumber], MotorType.kBrushless);
    angleMotor = new WPI_TalonSRX(ANGLE_MOTOR_PORTS[moduleNumber]);
    angleEncoder = new AnalogInput(ANGLE_ENCODER_PORTS[moduleNumber]);

    // NOTE: PIDController deals in native units
    // NOTE: positive rotation is counter-clockwise
    // initialize a pid controller using the super constructor
    // make continuous input enabled, since the motor can rotate past the extreme encoder count values

    angleFeedforward = new SimpleMotorFeedforward(ANGLE_FF_KS, ANGLE_FF_KV);
    anglePID = new ProfiledPIDController(
      ANGLE_PID_KP, 
      ANGLE_PID_KI, 
      ANGLE_PID_KD,
      new TrapezoidProfile.Constraints(
        MAX_MODULE_ANGULAR_SPEED, 
        MAX_MODULE_ANGULAR_ACCELERATION
      )
    );
    anglePID.enableContinuousInput(0, ANGLE_ENCODER_CPR);
    anglePID.setTolerance(ANGLE_PID_TOLERANCE);

    speedPID = new PIDController(SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD);

    if (moduleNumber == 1) {
      angleMotor.setInverted(InvertType.InvertMotorOutput);
    } else {
      angleMotor.setInverted(InvertType.None);
    }

  }



  /**
   * This method is called by the managing swerve drivetrain
   * @param setpoint the desired angle of the module in encoder ticks
   * @param speed the desired speed of the module in meters per second
   */
  public void drive(double setpoint, double speed) {
    setAngleSetpoint(setpoint);

    if (setpointAdjustmentNecessary()) {
      flipAngleSetpoint();
      speed *= -1;
    }

    double anglePIDOutput = anglePID.calculate(getAngle(), getAngleSetpoint());
    double angleFFOutput = angleFeedforward.calculate(anglePID.getSetpoint().velocity);

    if (anglePID.atGoal() || speed == 0.0)
      setSpeed(speed);

    angleMotor.setVoltage(angleFFOutput + anglePIDOutput);
  }



  /**
   * Sets the speed of the wheel
   * @param metersPerSecond how fast the wheel should rotate in meters per second
   */
  public void setSpeed(double metersPerSecond) {
    // rpm of motor = (distance / seconds) / (distance / rot_wheel) * (seconds / minute) * (rot_motor / rot_wheel)
    double setpoint = metersPerSecond / WHEEL_CIRCUMFERENCE * Constants.SECONDS_PER_MINUTE * DRIVE_GEAR_RATIO;
    double measurement = speedMotor.getEncoder().getVelocity();

    double output = speedPID.calculate(measurement, setpoint);

    // SmartDashboard.putNumber("Error " + moduleNumber, setpoint - measurement);
    // // TODO: added for armstrong
    // // scale input for testing; armstrong can't easily use above 2 lines bcs no falcons
    // // the scale is from 0 to MAX_WHEEL_SPEED usually, must be scaled to 0 to 1
    // metersPerSecond /= Constants.Swerve.MAX_WHEEL_SPEED;
    
    speedMotor.setVoltage(output);

    setSpeed = metersPerSecond;
  }



  /**
   * @param encoderCounts number of encoder counts
   * @return absolute encoder counts converted to radians
   */
  public static double nativeToRadians(double encoderCounts) {
    return encoderCounts * 2 * Math.PI / ANGLE_ENCODER_CPR;
  }



  /**
   * 
   * @param radians number of radians
   * @return radians converted to absolute encoder counts
   */
  public static double radiansToNative(double radians) {
    return radians / (2 * Math.PI) * ANGLE_ENCODER_CPR;
  }



  /**
   * The maximum angle the swerve module should have to travel to get to the right orientation
   * is 90 degrees, π / 2 radians, etc. If traveling to the setpoint would require exceeding this 
   * amount, it would be faster to rotate to the angle opposite the setpoint. This method checks
   * whether this adjustment is needed.
   * @return whether this setpoint needs adjustment
   */
  public boolean setpointAdjustmentNecessary() {
    double currentSetpoint = getAngleSetpoint();
    double currentPosition = angleEncoder.getAverageVoltage();

    // subtract the current position from the current setpoint to obtain the angle difference 
    // the easiest way to determine whether the angle difference exceeds 1/4 a rotation is to test 
    // whether the cosine of the angle is negative after converting the angle difference to radians
    // this method makes it easier to handle the continuous nature of angles
    double absAngleDifference = nativeToRadians(Math.abs(currentSetpoint - currentPosition));

    if (Math.cos(absAngleDifference) < 0) {
      return true;
    }
    
    // if the angle difference was under a quarter turn, the setpoint doesn't need to change 
    return false;
  }



  /**
   * If the setpoint needs to be adjusted so the module takes a faster route to an angle,
   * this method can readjust the setpoint to be the opposite angle
   */
  public void flipAngleSetpoint() {
    double currentSetpoint = getAngleSetpoint();
    double halfEncoderCircle = ANGLE_ENCODER_CPR / 2;

    // we need to flip the setpoint by half a circle, but we need to keep it within the range of
    // valid angle encoder values, so we make checks to determine whether to add or subtract a half
    // circle
    if (currentSetpoint < halfEncoderCircle) {
      setAngleSetpoint(currentSetpoint + halfEncoderCircle);
    } else {
      setAngleSetpoint(currentSetpoint - halfEncoderCircle);
    }
  }



  /**
   * 
   * @return current angle setpoint of angle pid in encoder ticks
   */
  public double getAngleSetpoint() {
    return anglePID.getGoal().position;
  }



  /**
   * 
   * @param setpoint the desired angle of module in encoder ticks
   */
  public void setAngleSetpoint(double setpoint) {
    anglePID.setGoal(setpoint);
  }


  
  /**
   * 
   * @return the angle of the module in encoder ticks
   */
  public double getAngle() {
    // Return the process variable measurement here
    return angleEncoder.getAverageVoltage();
  }



  /**
   * 
   * @return the speed in meters per second the module is set to
   */
  public double getSetSpeed() {
    return setSpeed;
  }



  /**
   * This is different from the speed this module is set to!
   * @return the calculated speed of the module in meters per second
   */
  public double getActualSpeed() {
    // (distance / seconds) = (rot_motor / minute) / (rot_motor / rot_wheel) / (seconds / minute) * (distance / rot_wheel) 
    return speedMotor.getEncoder().getVelocity() / DRIVE_GEAR_RATIO / Constants.SECONDS_PER_MINUTE * WHEEL_CIRCUMFERENCE;
  }



  @Override
  public void periodic() {

  }
}
