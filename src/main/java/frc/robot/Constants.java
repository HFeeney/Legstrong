// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SPI.Port;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants { 
    public static final double METERS_PER_INCH = 0.0254;
    public static final double SECONDS_PER_MINUTE = 60;

    public static class Swerve {
        public static final int NUM_MODULES = 4;
        
        // in meters
        public static final double WHEEL_BASE = 0.54;
        public static final double TRACK_WIDTH = 0.54;
        public static final double WHEEL_DIAMETER = 3.75 * METERS_PER_INCH;
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

        // in meters per second
        public static final double MAX_WHEEL_SPEED = 6; // TODO: unofficial number

        // in radians per second
        public static final double MAX_ANGULAR_SPEED = 6; // TODO: unoffical number
        
        // the motor ports should be in the order FL, FR, BL, BR
        public static final int[] SPEED_MOTOR_PORTS = {4, 5, 6, 7};
        public static final int[] ANGLE_MOTOR_PORTS = {0, 1, 2, 3};
        public static final int[] ANGLE_ENCODER_PORTS = {3, 0, 2, 1};
        public static final Port GYRO_PORT = Port.kMXP;
        
        // in encoder counts
        // the number that must be added to the setpoint of the module's rotation (one per module)
        // i.e. the value of the absolute encoder when the module is straight
        public static final double[] ANGLE_ENCODER_OFFSETS = {0.963, 2.216, 0.820, 4.733};

        // in encoder counts per revolution
        // CCW from above is positive direction
        public static final double ANGLE_ENCODER_CPR = 4.927;

        // in counts per revolution, this is the NEO integrated hall encoder
        public static final double SPEED_ENCODER_CPR = 42;
        
        // These are gear ratios, the number of rotations of driving gear per rotation of driven gear
        public static final double DRIVE_GEAR_RATIO = 5.4 / 1;
        public static final double TURN_GEAR_RATIO = 16 / 1;
        
        // find meters per encoder count using 
        // Circumference / Gear Ratio / Counts Per Motor Revolution = 
        // meters per rev wheel / rev motor per rev wheel / counts per rev motor
        public static final double METERS_PER_COUNT =  WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO / SPEED_ENCODER_CPR;

        public static final double ANGLE_PID_KP = 15;
        public static final double ANGLE_PID_KI = 0.0;
        public static final double ANGLE_PID_KD = 0.0;
        public static final double ANGLE_PID_TOLERANCE = 0.2;
        // in encoder ticks per second
        public static final double MAX_MODULE_ANGULAR_SPEED = 9.8; // 5;
        // in encoder ticks per second per second
        public static final double MAX_MODULE_ANGULAR_ACCELERATION = 78; // 9;

        public static final double SPEED_PID_KP = 0.002;
        public static final double SPEED_PID_KI = 0.0;
        public static final double SPEED_PID_KD = 0.0;

        public static final double ANGLE_FF_KS = 0.004;
        public static final double ANGLE_FF_KV = 1.889;

        // translation 2d considers the front of the robot as the positive x direction
        // and the left of the robot as the positive y direction
        public static final Translation2d FRONT_LEFT_MODULE_POSITION = new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d FRONT_RIGHT_MODULE_POSITION = new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
        public static final Translation2d BACK_LEFT_MODULE_POSITION = new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
        public static final Translation2d BACK_RIGHT_MODULE_POSITION = new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

        // TODO: what is this really?
        public static final Pose2d INITAL_POSE = new Pose2d(0, 0, new Rotation2d(0));
    }
}
