// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

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
public final class Config {

    /** Drivetrain Motor Data */
    public static int LEFT_MASTER = 1;
    public static int RIGHT_MASTER = 2;
    public static int LEFT_SLAVE = 3;
    public static int RIGHT_SLAVE = 4;

    public static int PIGEON_ID = 10;
    
    /** Drivetrain inversions */
    public static boolean INVERT_LEFT_MASTER = false;
    public static boolean INVERT_RIGHT_MASTER = false;
    public static boolean INVERT_LEFT_SLAVE = false;
    public static boolean INVERT_RIGHT_SLAVE = false;

    public static boolean INVERT_LEFT_ENCODER = false;
    public static boolean INVERT_RIGHT_ENCODER = false;
    public static boolean DRIVETRAIN_INVERT_DIFFERENTIALDRIVE = false;

    /** 
     * Drivetrain data 
     */

    // TrackWidth-> Distance between the left and right side of the drivetrain. Crucial in DifferentialDriveKinematics, see below.
    public static double trackWidth = 0.65;

    // Kinematics-> Converts a velocity & angular velocity to left velocity and right velocity. This value can affect how accurate
    //                  the robot does turns.
    public static DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(trackWidth);

    // Diameter of wheel & ticksPerRevolution used to convert meters into encoder ticks
    public static double drivetrainWheelDiameter = 0.1524; // 6 inches = 0.1524 meters
    public static double ticksPerRevolution = 4096;

    /** 
     * Characterization data
     */
    // ksVolts -> adds +ksVolts or -ksVolts to overcome static friction in the direction of motion.
    // kvVoltSecondsPerMeter -> Adds the values number of volts for every meter per second of velocity desired.
    // kaVoltSecondsSquaredPerMeter -> Adds the values number of volts for every meter per second squared of acceleration desired.
    public static double ksVolts = 0;
    public static double kvVoltSecondsPerMeter = 0;
    public static double kaVoltSecondsSquaredPerMeter = 0;

    // P Gain -> Number of ticks/100ms to apply for every ticks/100ms of error
    public static double kRamsetePGain = 0;

    /** 
     * Generating Trajectories Data
     */
    // MaxVelocity -> When it plans the trajectories, it will plan to go to the given max velocity.
    // MaxAcceleration -> When it plans the trajectories, it will ramp up/ramp down by this amount.
    //                  It will not plan to accelerate faster than this value.
    public static double kMaxSpeedMetersPerSecond = 1.0;
    public static double kMaxAccelerationMetersPerSecondSquared = 1.0;

    // voltageConstraint -> It will use this voltage constraint to make sure that the desired voltage given to a specific motor is achievable.
    //                          It will not allow any planned trajectory to tell a motor to go more than 10 volts.
    //                          This number is not 12 volts because it needs room to compensate.
    public static TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Config.ksVolts,
                            Config.kvVoltSecondsPerMeter, Config.kaVoltSecondsSquaredPerMeter), Config.differentialDriveKinematics, 10); 

    // TrajectoryConfig -> This object holds max velocity, max acceleration, kinematics to make sure those are obeyed and constraints.
    public static TrajectoryConfig trajectoryConfig = new TrajectoryConfig(kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(differentialDriveKinematics).addConstraint(autoVoltageConstraint); 

    /** Ramsete Logging */
    // Folder for Ramsete's logging data, /U means usb port closest to centre of Robo Rio
    public static String usbLogFolder = "/U/data_captures/";

    /** Generic Talon Settings */
    public static int TALON_PRIMARY_PID = 0;
    public static int TALON_AUXILIARY_PID = 1;
    
    /** Generic Can Settings */
    public static int CAN_TIMEOUT_LONG = 100;
    public static int CAN_TIMEOUT_SHORT = 10;

}
