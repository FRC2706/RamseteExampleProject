// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.FusionStatus;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.CTREUnits;
import frc.robot.config.Config;

public class DriveBase extends SubsystemBase {

    private DriveBase currentInstance;

    private WPI_TalonSRX leftMaster, rightMaster;
 
    private DifferentialDrive differentialDrive; 

    private DifferentialDriveOdometry odometry; 
    private SimpleMotorFeedforward feedforward;

    private Timer timer;

    private double prevTime = 0;
    private double prevLeftVelocity = 0;
    private double prevRightVelocity = 0;

    public DriveBase getInstance() {
        if (currentInstance == null) {
            currentInstance = new DriveBase();
        }
        return currentInstance;
    }

    private DriveBase() {
        leftMaster = new WPI_TalonSRX(Config.LEFT_MASTER);
        rightMaster = new WPI_TalonSRX(Config.RIGHT_MASTER);

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getCurrentAngle())); 
        feedforward = new SimpleMotorFeedforward(Config.ksVolts, Config.kvVoltSecondsPerMeter, Config.kaVoltSecondsSquaredPerMeter);

        timer.reset();
        timer.start();

        differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
        differentialDrive.setRightSideInverted(Config.DRIVETRAIN_INVERT_DIFFERENTIALDRIVE);
        
    }

    /**
     * Get the fused heading from the pigeon
     */
    private double getCurrentAngle() {
        // Get the current angle from the gyro (pigeon fused heading)
        return 0.0; 
    }

    /**
     * Get the encoder data in meters
     */
    private double getLeftPosition() {
        return CTREUnits.talonPosistionToMeters(leftMaster.getSelectedSensorPosition());
    }

    /**
     * Get the encoder data in meters
     */
    private double getRightPosition() {
        return CTREUnits.talonPosistionToMeters(rightMaster.getSelectedSensorPosition());

    }

    @Override
    public void periodic() {

        /** 
         * Give heading from gyro and encoder data in meters to odometry to calculate a new robot pose.
         */
        odometry.update(Rotation2d.fromDegrees(getCurrentAngle()), getLeftPosition(), getRightPosition());
    }

    /**
     * Returns the a Pose2d of the current robot location
     * 
     * Odometry calculates a new pose every robot cycle and stores
     * the value so this method is only reading the stored value.
     * This means it already does only 1 hardware read every cycle instead of 
     * many things calling hardware redundantly.
     * 
     * @param Pose2d the current pose of the robot
     */
    public Pose2d getPose() { 
        return odometry.getPoseMeters();
    }

    /**
     * This method will return the heading from odometry
     * 
     * Odometry keeps track of the gyro heading and in relation to
     * the value it was reset to using an offset so it's important to ask
     * the odometry for the rotation instead of directly from the gyro.
     * 
     * @param Rotation2d The heading. Rotation2d has a .getDegrees() method.
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Set the odometry to a given pose.
     * 
     * Necessary to do at the beginning of a match.
     * 
     * Very important to reset the encoders to 0 when resetting odometry.
     * 
     * @param newPose New pose to set odometry to.
     */
    public void resetPose(Pose2d newPose) {
        ErrorCode leftError = leftMaster.setSelectedSensorPosition(0, Config.TALON_PRIMARY_PID, Config.CAN_TIMEOUT_LONG);
        ErrorCode rightError = rightMaster.setSelectedSensorPosition(0, Config.TALON_PRIMARY_PID, Config.CAN_TIMEOUT_LONG);

        odometry.resetPosition(newPose, Rotation2d.fromDegrees(getCurrentAngle()));
    }


    public void setVelocity(double leftVelocity, double rightVelocity) {
        // Calculate time since last call (deltaTime)
        double currentTime = timer.get();
        double deltaTime = timer.get() - prevTime;
        prevTime = currentTime;

        // Throw out a time longer than a few robot cycles
        if (deltaTime > 0.1)  {
            return;
        }

        // Calculate acceleration
        double leftAcceleration = (leftVelocity - prevLeftVelocity) / deltaTime;
        double rightAcceleration = (rightVelocity - prevRightVelocity) / deltaTime;

        // Calculate feed forward value
        double leftFeedforward = feedforward.calculate(leftVelocity, leftAcceleration);
        double rightFeedforward = feedforward.calculate(rightVelocity, rightAcceleration);

        // Give the feed forward value and velocity set point to the talons
        leftMaster.set(ControlMode.Velocity, CTREUnits.metersPerSecondToTalonVelocity(leftVelocity),
                DemandType.ArbitraryFeedForward, leftFeedforward / 12.0);

        rightMaster.set(ControlMode.Velocity, CTREUnits.metersPerSecondToTalonVelocity(rightVelocity),  
                DemandType.ArbitraryFeedForward, rightFeedforward / 12.0); 

        // Make sure motor safety knows the motors are being used
        differentialDrive.feed();
    }

}
