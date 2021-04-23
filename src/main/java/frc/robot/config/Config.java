// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
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
    
    // INVERSIONS
    public static boolean DRIVETRAIN_INVERT_DIFFERENTIALDRIVE = false;

    /** Drivetrain data */
    public static double trackWidth = 0.65;
    public static DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(trackWidth);
    public static double drivetrainWheelDiameter = 0.1524; // 6 inches in meters
    public static double ticksPerRevolution = 4096;

    /** Characterization data */
    public static double kRamsetePGain = 0;
    public static double ksVolts = 0;
    public static double kvVoltSecondsPerMeter = 0;
    public static double kaVoltSecondsSquaredPerMeter = 0;

    /** Ramsete Data */
    public static TrajectoryConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Config.ksVolts,
                            Config.kvVoltSecondsPerMeter, Config.kaVoltSecondsSquaredPerMeter), Config.differentialDriveKinematics, 10); 

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
